#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16



class RoverController:
    def __init__(self):
        rospy.init_node('rover_controller', anonymous=True)

        # Publishers for steer, choice, and pwm topics
        
        self.steer_publisher = rospy.Publisher('/rover/steer', Int16, queue_size=10)
        self.choice_publisher = rospy.Publisher('/rover/choice', Int16, queue_size=10)
        self.pwm_publisher = rospy.Publisher('/rover/pwm', Int16, queue_size=10)
        self.flag1 = 0
        self.flag2 = 0

        # Subscriber to cmd_vel
        self.cmd_vel_subscriber = rospy.Subscriber('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        """
        Callback to process Twist messages and convert them into sequential rover commands.
        Each command (steer, choice, pwm) is published separately with appropriate delays.
        """
        linear_x = msg.linear.x  # Forward/backward motion
        angular_z = msg.angular.z  # Rotational motion

        if linear_x > 0:  # Forward
            rospy.loginfo("Moving forward")
            self.publish_steer(2)  # Normal mode
            rospy.sleep(0.1)
            self.publish_choice(1)  # Forward
            rospy.sleep(0.1)
            self.publish_pwm(100)

        elif linear_x < 0:  # Backward
            rospy.loginfo("Moving backward")
            self.publish_steer(2)  # Normal mode
            rospy.sleep(0.1)
            self.publish_choice(2)  # Backward
            rospy.sleep(0.1)
            self.publish_pwm(100)

        elif angular_z > 0:  # Left spot turn
            rospy.loginfo("Spot turning left")
            if self.flag1 == 0:
                self.publish_steer(1)  
                rospy.sleep(3)  # Wait for 3 seconds
                self.flag1 = 1
                self.flag2 = 1
            else:
                self.publish_choice(3)  # Left
                rospy.sleep(0.1)
                self.publish_pwm(100)

        elif angular_z < 0:  # Right spot turn
            rospy.loginfo("Spot turning right")
            if self.flag1 == 0:
                self.publish_steer(1)  # Spot turn mode
                rospy.sleep(3)  # Wait for 3 seconds
                self.flag1 = 1
                self.flag2 = 1
            else:
                self.publish_choice(4)  # Right
                rospy.sleep(0.1)
                self.publish_pwm(100)

        else:  # Stop
            rospy.loginfo("Stopping")
            self.publish_steer(2)  # Normal mode
            rospy.sleep(0.1)
            self.publish_choice(1)  # Stop
            rospy.sleep(0.1)
            self.publish_pwm(0)
            self.flag1 = 0
            
        if angular_z == 0 and self.flag2 == 1:
            self.publish_steer(2)  # Normal mode
            rospy.sleep(3)
            self.flag2 = 0

    def publish_steer(self, steer_value):
        """
        Publishes to the /rover/steer topic.
        """
        self.steer_publisher.publish(steer_value)
        rospy.loginfo(f"Published steer: {steer_value}")

    def publish_choice(self, choice_value):
        """
        Publishes to the /rover/choice topic.
        """
        self.choice_publisher.publish(choice_value)
        rospy.loginfo(f"Published choice: {choice_value}")

    def publish_pwm(self, pwm_value):
        """
        Publishes to the /rover/pwm topic.
        """
        self.pwm_publisher.publish(pwm_value)
        rospy.loginfo(f"Published pwm: {pwm_value}")

    def run(self):
        """
        Keeps the node running.
        """
        rospy.spin()

if __name__ == "__main__":
    controller = RoverController()
    controller.run()
