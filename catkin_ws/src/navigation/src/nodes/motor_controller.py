#! /usr/bin/env python

'''
This node controls the motors (inner controller)
Subscribed topics:
    measured_wheel_speeds     [asclinc_pkg/LeftRightFloat32]
    reference_wheel_speeds    [asclinc_pkg/LeftRightFloat32]
    plot                      [Bool]
Published topics:
    /asc/set_motor_duty_cycle [asclinc_pkg/LeftRightFloat32]
'''

import rospy
from std_msgs.msg import Bool
from asclinic_pkg.msg import LeftRightFloat32, LeftRightFloat32
import matplotlib.pyplot as plt

K_u = 200.0
T_u = 20 / 3.0

class MotorController:
    def __init__(self):
        # Get user parameter
        self.verbosity = rospy.get_param('~verbosity', 0)
        self.plot = rospy.get_param('~plot', 0)

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

        # PID parameters (Tune these for your specific motors and robot)
        self.Kp = 20
        self.Ki = 20
        self.Kd = 0

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

        self.integral_left += error_left
        self.integral_right += error_right

        derivative_left = error_left - self.last_error_left
        derivative_right = error_right - self.last_error_right

        duty_cycle_left = self.Kp * error_left + self.Ki * self.integral_left + self.Kd * derivative_left
        duty_cycle_right = self.Kp * error_right + self.Ki * self.integral_right + self.Kd * derivative_right

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
