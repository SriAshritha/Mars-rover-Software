#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time


class NodeStatus:
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"


class Node:
    def run(self):
        raise NotImplementedError("run() must be implemented in subclass")


class SequenceNode(Node):
    def __init__(self, children):
        self.children = children

    def run(self):
        for child in self.children:
            status = child.run()
            if status != NodeStatus.SUCCESS:
                return status
        return NodeStatus.SUCCESS

class ParallelNode(Node):
    def __init__(self, children, success_threshold, failure_threshold):
         
        self.children = children
        self.success_threshold = success_threshold
        self.failure_threshold = failure_threshold

    def run(self):
         
        success_count = 0
        failure_count = 0

        for child in self.children:
            status = child.run()
            if status == NodeStatus.SUCCESS:
                success_count += 1
            elif status == NodeStatus.FAILURE:
                failure_count += 1

            # Check thresholds after each child's status is evaluated
            if success_count >= self.success_threshold:
                return NodeStatus.SUCCESS
            if failure_count >= self.failure_threshold:
                return NodeStatus.FAILURE

        return NodeStatus.RUNNING


class FallbackNode(Node):
    def __init__(self, children):
        self.children = children

    def run(self):
        for child in self.children:
            status = child.run()
            if status == NodeStatus.SUCCESS:
                return status
        return NodeStatus.FAILURE


class MoveStraight(Node):
    def __init__(self):
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)

    def run(self):
        rospy.loginfo("Moving straight...")
        twist = Twist()
        twist.linear.x = 0.5
        self.publisher.publish(twist)
        return NodeStatus.RUNNING


class ArrowDetected(Node):
    def __init__(self):
        self.arrow_direction = None
        rospy.Subscriber("/arrow_direction", Int32, self.arrow_callback)
        self.move_left = MoveLeft()
        self.move_right = MoveRight()

    def arrow_callback(self, msg):
        self.arrow_direction = msg.data

    def run(self):
        if self.arrow_direction == 0:
            rospy.loginfo("Arrow detected pointing left.")
            return self.move_left.run()
        elif self.arrow_direction == 1:
            rospy.loginfo("Arrow detected pointing right.")
            return self.move_right.run()
        else:
            rospy.loginfo("No arrow detected.")
            return NodeStatus.RUNNING


class MoveLeft(Node):
    def __init__(self):
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)

    def run(self):
        rospy.loginfo("Rotating left...")
        twist = Twist()
        twist.angular.z = 0.25
        duration = 6.28  # Adjust the duration for 90-degree rotation
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            rospy.sleep(0.1)
        twist.angular.z = 0
        self.publisher.publish(twist)
        return NodeStatus.SUCCESS


class MoveRight(Node):
    def __init__(self):
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)

    def run(self):
        rospy.loginfo("Rotating right...")
        twist = Twist()
        twist.angular.z = -0.25
        duration = 6.28  # Adjust the duration for 90-degree rotation
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            rospy.sleep(0.1)
        twist.angular.z = 0
        self.publisher.publish(twist)
        return NodeStatus.SUCCESS


class MoveTowardsArrow(Node):
    def __init__(self):
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)

    def run(self):
        rospy.loginfo("Moving towards the arrow...")
        twist = Twist()
        twist.linear.x = 0.5
        self.publisher.publish(twist)
        return NodeStatus.RUNNING


class GetDistanceData(Node):
    def __init__(self):
        self.min_distance = float('inf')
        rospy.Subscriber('/arrow_distance', Int32, self.distance_callback)

    def distance_callback(self, msg):
        self.min_distance = msg.data
        rospy.loginfo(f"Distance to arrow: {self.min_distance:.2f} cm")

    def run(self):
        if self.min_distance <= 150.00:
            rospy.loginfo("Reached arrow.")
            return NodeStatus.SUCCESS
        return NodeStatus.RUNNING


class StopAtArrow(Node):
    def __init__(self):
        self.seconds = 10

    def run(self):
        rospy.loginfo(f"Stopping for {self.seconds} seconds...")
        rospy.sleep(self.seconds)
        return NodeStatus.SUCCESS


class TurnInArrowDirection(Node):
    def __init__(self):
        self.arrow_direction = None
        rospy.Subscriber("/arrow_direction", Int32, self.arrow_callback)
        self.move_left = MoveLeft()
        self.move_right = MoveRight()

    def arrow_callback(self, msg):
        self.arrow_direction = msg.data

    def run(self):
        if self.arrow_direction == 0:
            rospy.loginfo("Turning left after stopping.")
            return self.move_left.run()
        elif self.arrow_direction == 1:
            rospy.loginfo("Turning right after stopping.")
            return self.move_right.run()
        return NodeStatus.FAILURE


# Main Function
if __name__ == "__main__":
    rospy.init_node('robot_behavior_tree', anonymous=True)

    behavior_tree = FallbackNode([
        SequenceNode([
            FallbackNode([
                ArrowDetected(),
                TurnUntilVisible()
            ]),
            MoveTowardsArrow(),
            FallbackNode([
                ParallelNode(children=[GetDistanceData(), MoveTowardsArrow()], success_threshold=2, failure_threshold=2),
                SequenceNode([
                    ParallelNode(children=[CheckOtherSensors(), GetDistanceData(), MoveTowardsArrow()], success_threshold=3, failure_threshold=2)
                ])
            ]),
            SequenceNode([
                StopAtDistance(stop_distance=1.0),
                WaitForSeconds(10),
                FallbackNode([
                    TurnInArrowDirection(),
                    ParallelNode(children=[TurnInArrowDirection()], success_threshold=1, failure_threshold=1)
                ]),
                CheckParallelDirection()
            ])
        ]),
        SequenceNode([
            TurnUntilVisible(),
            MoveTowardsArrow(),
            FallbackNode([
                ParallelNode(children=[MoveTowardsArrow(), GetDistanceData()], success_threshold=2, failure_threshold=2),
                SequenceNode([
                    ParallelNode(children=[CheckOtherSensors(), GetDistanceData(), MoveTowardsArrow()], success_threshold=3, failure_threshold=2)
                ])
            ]),
            SequenceNode([
                StopAtDistance(stop_distance=1.0),
                WaitForSeconds(10),
                FallbackNode([
                    TurnInArrowDirection(),
                    ParallelNode(children=[TurnInArrowDirection()], success_threshold=1, failure_threshold=1)
                ]),
                CheckParallelDirection()
            ])
        ])
    ])

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        status = behavior_tree.run()
        rospy.loginfo(f"Behavior Tree Status: {status}")
        rate.sleep()

