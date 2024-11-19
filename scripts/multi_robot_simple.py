#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


class TurtlebotController:

    def __init__(self):
        rospy.init_node("turtlebot_controller", anonymous=True)
        self.cmd_vel_pub_tb3_0 = rospy.Publisher("tb3_0/cmd_vel", Twist, queue_size=10)
        self.cmd_vel_pub_tb3_1 = rospy.Publisher("tb3_1/cmd_vel", Twist, queue_size=10)
        self.cmd_vel_pub_tb3_2 = rospy.Publisher("tb3_2/cmd_vel", Twist, queue_size=10)
        self.cmd_vel_publishers = [self.cmd_vel_pub_tb3_0, self.cmd_vel_pub_tb3_1, self.cmd_vel_pub_tb3_2]
        self.rate = rospy.Rate(10)  # Set rate to 10 Hz
        rospy.sleep(1)  # Add delay to ensure publisher is set up properly

    def publish_twist(self, move_cmd, duration, robot_i):
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.cmd_vel_publishers[robot_i].publish(move_cmd)
            self.rate.sleep()

    def stop_turtlebot(self, robot_i):
        move_cmd = Twist()
        ######### Your code starts here #########
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        ######### Your code ends here #########
        self.publish_twist(move_cmd, 1, robot_i)  # Stop for 1 second

    def move_forward(self, robot_i):
        print("Moving forward...")
        move_cmd = Twist()
        ######### Your code starts here #########
        move_cmd.linear.x = 0.05  # Adjust speed as needed
        move_cmd.angular.z = 0.0
        ######### Your code ends here #########
        self.publish_twist(move_cmd, 10, robot_i)  # Move forward for 10 seconds

    def move_backward(self, robot_i):
        print("Moving backward...")
        move_cmd = Twist()
        ######### Your code starts here #########
        move_cmd.linear.x = -0.05  # Adjust speed as needed
        move_cmd.angular.z = 0.0
        ######### Your code ends here #########
        self.publish_twist(move_cmd, 10, robot_i)  # Move backward for 10 seconds

    def turn_left(self, robot_i):
        print("Turning left...")
        move_cmd = Twist()
        ######### Your code starts here #########
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.2  # Adjust angular speed as needed
        ######### Your code ends here #########
        self.publish_twist(move_cmd, 5, robot_i)  # Turn left for 5 seconds

    def turn_right(self, robot_i):
        print("Turning right...")
        move_cmd = Twist()
        ######### Your code starts here #########
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -0.2  # Adjust angular speed as needed
        ######### Your code ends here #########
        self.publish_twist(move_cmd, 5, robot_i)  # Turn right for 5 seconds

    def move_sequence(self):
        # Move forward
        self.move_forward(0)
        self.move_forward(1)

        # Turn left
        self.turn_left(0)
        self.turn_left(1)

        # Turn right
        self.turn_right(0)
        self.turn_right(1)

        # Move backward
        self.move_backward(0)
        self.move_backward(1)

        # Stop at the end of sequence
        self.stop_turtlebot(0)
        self.stop_turtlebot(1)


def main():
    controller = TurtlebotController()
    try:
        controller.move_sequence()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

