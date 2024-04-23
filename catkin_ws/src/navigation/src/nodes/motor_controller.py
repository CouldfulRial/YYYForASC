#! /usr/bin/env python

'''
This node controls the motors (inner controller)
Subscribed topics:
    measured_vel [geometry_msgs/Twist]
    ref_vel      [geometry_msgs/Twist]
Published topics:
    cmd_vel      [geometry_msgs/Twist]
'''

import rospy
from std_msgs.msg import Bool
from asclinic_pkg.msg import LeftRightFloat32, LeftRightFloat32
import matplotlib.pyplot as plt


class MotorController:
    def __init__(self):
        # Get user parameter
        self.verbosity = rospy.get_param('~verbosity', 'default_value')
        self.plot      = rospy.get_param('~plot', 'default_value')

        rospy.init_node('motor_controller')

        # Subscribers
        self.ref_speed_sub = rospy.Subscriber('reference_wheel_speeds', LeftRightFloat32, self.ref_speed_callback)
        self.mes_speed_sub = rospy.Subscriber('measured_wheel_speeds', LeftRightFloat32, self.mes_speed_callback)
        # self.plot_sub = rospy.Subscriber('plot', Bool, self.plot_callback)

        # Timer: Calls the timer_callback function at 20 Hz
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        # Publisher
        self.duty_cycle_pub = rospy.Publisher('/asc/set_motor_duty_cycle', LeftRightFloat32, queue_size=10)

        # Register the shutdown callback
        rospy.on_shutdown(self.shutdown_callback)
        if self.plot == 1:
            self.left_speed_list = []
            self.right_speed_list = []
            self.desired_left_speed_list = []
            self.desired_right_speed_list = []

        # Initialise speeds variables
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        self.desired_left_speed = 0.0
        self.desired_right_speed = 0.0

        # Integral and Derivative components
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.last_error_left = 0.0
        self.last_error_right = 0.0

    def timer_callback(self, event):
        # Controller execution callback
        # Error calculation
        error_left = self.desired_left_speed - self.current_left_speed
        error_right = self.desired_right_speed - self.current_right_speed

        # Store the speeds
        if self.plot == 1:
            self.left_speed_list.append(self.current_left_speed)
            self.right_speed_list.append(self.current_right_speed)
            self.desired_left_speed_list.append(self.desired_left_speed)
            self.desired_right_speed_list.append(self.desired_right_speed)

        duty_cycle_left, duty_cycle_right = self.controller(error_left, error_right)

        # Publish duty cycles
        self.duty_cycle_pub.publish(LeftRightFloat32(left=duty_cycle_left, right=duty_cycle_right))

        self.last_error_left = error_left
        self.last_error_right = error_right

        # Diaply in the console
        if self.verbosity == 1:
            rospy.loginfo("-"*25 + "Motor Controller" + "-"*25 +  
                        f"\nDesired Left Speed: {self.desired_left_speed:3.2f}, Desired Right Speed: {self.desired_right_speed:3.2f}\n" + 
                        f"Current Left Speed:   {self.current_left_speed:3.2f}, Current Right Speed: {self.current_right_speed:3.2f}\n" + 
                        f"Error_left:           {error_left:3.2f},              Error_right:         {error_right:3.2f}\n" + 
                        f"Current Left Speed:   {self.current_left_speed:3.2f}, Current Right Speed: {self.current_right_speed:3.2f}\n" + 
                        f"Left Duty Cycle:      {duty_cycle_left:3.2f},         Right Duty Cycle:    {duty_cycle_right:3.2f}\n")

    def controller(self, error_left, error_right):
        # PID parameters
        Kp = 20
        Ki = 20
        Kd = 0

        # I
        self.integral_left  += error_left
        self.integral_right += error_right

        # D
        derivative_left  = error_left  - self.last_error_left
        derivative_right = error_right - self.last_error_right

        duty_cycle_left  = Kp * error_left  + Ki * self.integral_left  + Kd * derivative_left
        duty_cycle_right = Kp * error_right + Ki * self.integral_right + Kd * derivative_right

        return duty_cycle_left, duty_cycle_right

    def ref_speed_callback(self, data):
        self.desired_left_speed = data.left
        self.desired_right_speed = data.right

    def mes_speed_callback(self, data):
        self.current_left_speed = data.left
        self.current_right_speed = data.right

    def shutdown_callback(self):
        rospy.loginfo("Shutting Down Motors....")
        self.duty_cycle_pub.publish(LeftRightFloat32(left=0, right=0))
        if self.plot == 1:
            self.plot_callback()

    def plot_callback(self):
        plt.plot(self.left_speed_list, label='Left Wheel Speed')
        plt.plot(self.right_speed_list, label='Right Wheel Speed')
        plt.plot(self.desired_left_speed_list, label='desired left')
        plt.plot(self.desired_right_speed_list, label='desired right')
        plt.xlabel('steps')
        plt.ylabel('speeds')
        plt.title("motor controller")
        plt.show()

if __name__ == '__main__':
    controller = MotorController()
    rospy.spin()
