#! /usr/bin/env python

'''
This node controls the motors (inner controller)
Subscribed topics:
    measured_wheel_speeds     [asclinc_pkg/LeftRightFloat32]
    reference_wheel_speeds    [asclinc_pkg/LeftRightFloat32]
Published topics:
    /asc/set_motor_duty_cycle [asclinc_pkg/LeftRightFloat32]
'''

import rospy
from std_msgs.msg import Float32
from asclinic_pkg.msg import LeftRightFloat32, LeftRightFloat32

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller')

        # Subscribers
        self.ref_speed_sub = rospy.Subscriber('reference_wheel_speeds', LeftRightFloat32, self.ref_speed_callback)
        self.mes_speed_sub = rospy.Subscriber('measured_wheel_speeds', LeftRightFloat32, self.mes_speed_callback)

        # Publisher
        self.duty_cycle_pub = rospy.Publisher('/asc/set_motor_duty_cycle', LeftRightFloat32, queue_size=10)

        # Current measured speeds
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0

        # PID parameters (Tune these for your specific motors and robot)
        self.Kp = 50.0
        self.Ki = 1
        self.Kd = 0.05

        # Integral and Derivative components
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.last_error_left = 0.0
        self.last_error_right = 0.0

        self.rate = rospy.Rate(10)  # 10 Hz

    def ref_speed_callback(self, data):
        desired_left_speed = data.left
        desired_right_speed = data.right

        # Error calculation
        error_left = desired_left_speed - self.current_left_speed
        error_right = desired_right_speed - self.current_right_speed

        self.integral_left += error_left
        self.integral_right += error_right

        derivative_left = error_left - self.last_error_left
        derivative_right = error_right - self.last_error_right

        duty_cycle_left = self.Kp * error_left + self.Ki * self.integral_left + self.Kd * derivative_left
        duty_cycle_right = self.Kp * error_right + self.Ki * self.integral_right + self.Kd * derivative_right
        # duty_cycle_left = self.Kp * error_left
        # duty_cycle_right = self.Kp * error_right

        # Publish duty cycles
        dc_msg = LeftRightFloat32()
        dc_msg.left = duty_cycle_left
        dc_msg.right = duty_cycle_right
        self.duty_cycle_pub.publish(dc_msg)

        self.last_error_left = error_left
        self.last_error_right = error_right

        # Diaply in the console
        rospy.loginfo("-"*50 + 
                      f"\nDesired Left Speed: {desired_left_speed:3.2f},      Desired Right Speed: {desired_right_speed:3.2f}\n" + 
                      f"Current Left Speed:   {self.current_left_speed:3.2f}, Current Right Speed: {self.current_right_speed:3.2f}\n" + 
                      f"Error_left:           {error_left:3.2f},              Error_right:         {error_right:3.2f}\n" + 
                      f"Current Left Speed:   {self.current_left_speed:3.2f}, Current Right Speed: {self.current_right_speed:3.2f}\n" + 
                      f"Left Duty Cycle:      {duty_cycle_left:3.2f},         Right Duty Cycle:    {duty_cycle_right:3.2f}\n")

        self.rate.sleep()

    def mes_speed_callback(self, data):
        self.current_left_speed = data.left
        self.current_right_speed = data.right

if __name__ == '__main__':
    controller = MotorController()
    rospy.spin()
