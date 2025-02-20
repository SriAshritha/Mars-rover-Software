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


class FallbackNode(Node):
    def __init__(self, children):
        self.children = children

    def run(self):
        for child in self.children:
            status = child.run()
            if status == NodeStatus.SUCCESS:
                return status
        return NodeStatus.FAILURE


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

            if success_count >= self.success_threshold:
                return NodeStatus.SUCCESS
            if failure_count >= self.failure_threshold:
                return NodeStatus.FAILURE

        return NodeStatus.RUNNING


class MoveLeft(Node):
    def __init__(self):
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)

    def run(self):
        rospy.loginfo("Rotating left 90 degrees...")
        twist = Twist()
        twist.angular.z = 0.25
        duration = 6.28
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            rospy.sleep(0.3)
        twist.angular.z = 0
        self.publisher.publish(twist)
        return NodeStatus.SUCCESS


class MoveRight(Node):
    def __init__(self):
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)

    def run(self):
        rospy.loginfo("Rotating right 90 degrees...")
        twist = Twist()
        twist.angular.z = -0.25
        duration = 6.28
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            rospy.sleep(0.3)
        twist.angular.z = 0
        self.publisher.publish(twist)
        return NodeStatus.SUCCESS


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
            return NodeStatus.FAILURE


class TurnUntilVisible(Node):
    def __init__(self):
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)

    def run(self):
        rospy.loginfo("Turning to find an arrow...")
        twist = Twist()
        twist.angular.z = 0.5
        duration = 5
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            rospy.sleep(0.3)
        twist.angular.z = 0
        self.publisher.publish(twist)
        return NodeStatus.SUCCESS


class GetDistanceData(Node):
    def __init__(self):
        self.min_distance = float('inf')
        rospy.Subscriber('/arrow_distance', Int32, self.distance_callback)
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)

    def distance_callback(self, msg):
        self.min_distance = msg.data
        rospy.loginfo(f"Distance: {self.min_distance:.2f} cm")

    def run(self):
        rospy.loginfo(f"Distance to arrow: {self.min_distance:.2f} cm")
        rospy.loginfo("Moving towards the arrow...")
        twist = Twist()
        twist.linear.x = 0.5
        if self.min_distance <= 150:
            twist.linear.x = 0
            twist.angular.z = 0
            self.publisher.publish(twist)
            rospy.loginfo("arrived")
            return NodeStatus.SUCCESS
        self.publisher.publish(twist)
        rospy.sleep(0.3)
        return NodeStatus.RUNNING


class WaitForSeconds(Node):
    def __init__(self, seconds):
        self.seconds = seconds
        self.publisher = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)

    def run(self):
        rospy.loginfo(f"Waiting for {self.seconds} seconds...")
        twist = Twist()
        twist.linear.x = 0
        self.publisher.publish(twist)
        rospy.sleep(self.seconds)
        return NodeStatus.SUCCESS


class TurnInArrowDirection(Node):
    def run(self):
        rospy.loginfo("Turning in arrow direction...")
        return NodeStatus.SUCCESS


# Main Function
if __name__ == "__main__":
    rospy.init_node('robot_behavior_tree', anonymous=True)

    behavior_tree = SequenceNode([
        GetDistanceData(),
        SequenceNode([
            WaitForSeconds(10),
            FallbackNode([
                ArrowDetected(),
                TurnInArrowDirection()
            ])
        ]),
        GetDistanceData(),
        SequenceNode([
            WaitForSeconds(10),
            FallbackNode([
                ArrowDetected(),
                TurnInArrowDirection()
            ])
        ])
    ])

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        status = behavior_tree.run()
        rospy.loginfo(f"Behavior Tree Status: {status}")
        rate.sleep()

