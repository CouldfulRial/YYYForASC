#! /usr/bin/env python
'''
This node controls the motors (inner controller)
Subscribed topics:
    mes_speeds                [asclinic_pkg/LeftRightFloat32]
    cmd_speeds                [asclinic_pkg/LeftRightFloat32]
Published topics:
    /asc/set_motor_duty_cycle [asclinic_pkg/LeftRightFloat32]
'''

import rospy
import rospkg
import datetime
import csv
from math import pi
from geometry_msgs.msg import Twist
from asclinic_pkg.msg import LeftRightFloat32, LeftRightFloat32
import matplotlib.pyplot as plt

# Constatns
TIME_STEP = 0.1  # s
DEBUG_LENGTH = 5  # s

# Ziegler–Nichols method parameters
KU_LEFT = 15
KU_RIGHT = 15
TU = 0.2

class MotorController:
    def __init__(self):
        # Initialise node
        self.node_name = 'motor_controller'
        rospy.init_node(self.node_name)

        # Get user parameter
        self.parm  = rospy.get_param(self.node_name)
        self.verbosity = self.parm["verbosity"]
        self.save_data = self.parm["save_data"]
        self.debug = self.parm["debug"]  # If debug, only run for DEBUG_LENGTH seconds

        # Subscribers
        # self.ref_speed_sub = rospy.Subscriber('cmd_speeds', Twist, self.ref_speeds_callback)
        self.mes_speed_sub = rospy.Subscriber('mes_speeds', LeftRightFloat32, self.mes_speeds_callback)

        # Timer: Calls the timer_callback function at 10 Hz
        self.timer = rospy.Timer(rospy.Duration(TIME_STEP), self.timer_callback)

        # Publisher
        self.duty_cycle_pub = rospy.Publisher('/asc/set_motor_duty_cycle', LeftRightFloat32, queue_size=10)

        # Register the shutdown callback
        rospy.on_shutdown(self.shutdown_callback)
        if self.save_data == 1:
            self.time_list = [0]
            self.left_speed_list = [0]
            self.right_speed_list = [0]
            self.desired_left_speed_list = [0]
            self.desired_right_speed_list = [0]

        # Initialise variables
        # left
        self.current_left_speed = 0.0
        self.desired_left_speed = -pi/2
        self.integral_left = 0.0
        self.integral_right = 0.0
        # right
        self.current_right_speed = 0.0
        self.desired_right_speed = pi/2
        self.last_error_left = 0.0
        self.last_error_right = 0.0

        # Get save path. Set the file name to the intial time
        self.save_path = self.get_save_path()
        intial_time = datetime.datetime.now()
        self.formatted_time = intial_time.strftime('%y%m%d_%H_%M_%S')

    def timer_callback(self, event):
        # Controller algorithm
        # Error calculation
        error_left = self.desired_left_speed - self.current_left_speed
        error_right = self.desired_right_speed - self.current_right_speed

        # Store the speeds data
        if self.save_data == 1:
            self.time_list.append(self.time_list[-1] + TIME_STEP)
            self.left_speed_list.append(self.current_left_speed)
            self.right_speed_list.append(self.current_right_speed)
            self.desired_left_speed_list.append(self.desired_left_speed)
            self.desired_right_speed_list.append(self.desired_right_speed)
        if self.debug == 1 and self.time_list[-1] >= DEBUG_LENGTH:
            rospy.signal_shutdown("Debugging finished")

        duty_cycle_left, duty_cycle_right = self.controller(error_left, error_right)
        duty_cycle_left = self.saturation(duty_cycle_left, 20)
        duty_cycle_right = self.saturation(duty_cycle_right, 20)

        # Diaply in the console
        if self.verbosity == 1:
            rospy.loginfo("-"*25 + "Motor Controller" + "-"*25 +  
                        f"\nDesired Left Speed: {self.desired_left_speed:3.5f}, Desired Right Speed: {self.desired_right_speed:3.5f}\n" + 
                        f"Current Left Speed:   {self.current_left_speed:3.5f}, Current Right Speed: {self.current_right_speed:3.5f}\n" + 
                        f"Error_left:           {error_left:3.5f},              Error_right:         {error_right:3.5f}\n" + 
                        f"Left Duty Cycle:      {duty_cycle_left:3.5f},         Right Duty Cycle:    {duty_cycle_right:3.5f}\n")
            
        # Publish duty cycles
        self.duty_cycle_pub.publish(LeftRightFloat32(left=duty_cycle_left, right=duty_cycle_right))

        self.last_error_left = error_left
        self.last_error_right = error_right

    def controller(self, error_left, error_right):
        # PID parameters
        P = 2
        I = 0
        D = 0
        # Left
        self.Kp_l = 50
        self.Ki_l = I * KU_LEFT / TU
        self.Kd_l = D * KU_LEFT * TU
        # Right
        self.Kp_r = 50
        self.Ki_r = I * KU_RIGHT / TU
        self.Kd_r = D * KU_RIGHT * TU

        # I
        self.integral_left  += error_left
        self.integral_right += error_right

        # D
        derivative_left  = error_left  - self.last_error_left
        derivative_right = error_right - self.last_error_right

        duty_cycle_left  = self.Kp_l * error_left  + self.Ki_l * self.integral_left  + self.Kd_l * derivative_left
        duty_cycle_right = self.Kp_r * error_right + self.Ki_r * self.integral_right + self.Kd_r * derivative_right

        return duty_cycle_left, duty_cycle_right

    def ref_speeds_callback(self, data):
        self.desired_left_speed, self.desired_right_speed = data.left, data.right

    def mes_speeds_callback(self, data):
        self.current_left_speed, self.current_right_speed = data.left, data.right


    ##############################################################################################################
    ############################## The following codes are not relevant to algorithm ##############################
    ##############################################################################################################
    def shutdown_callback(self):
        rospy.loginfo(f"Shutting down node {self.node_name}....")
        # while self.current_left_speed != 0 or self.current_right_speed != 0:
        self.duty_cycle_pub.publish(LeftRightFloat32(left=0, right=0))
        if self.save_data == 1:
            rospy.loginfo(f"Saving Data To {self.save_path}....")
            self.plot()
            self.data()
            rospy.loginfo("Saving Data Completed....")

    def plot(self):
        # set the font size
        plt.rcParams.update({'font.size': 5})

        # plot left
        ax1 = plt.subplot(2, 1, 1)
        plt.plot(self.time_list, self.left_speed_list, label='Left Wheel Speed')
        plt.plot(self.time_list, self.desired_left_speed_list, label='desired left')
        plt.ylabel('Left Wheel Speed (rad/s)')
        plt.title(f"Left Motor Controller, Kp_l = {self.Kp_l:2.2f}, Ki_l = {self.Ki_l:2.2f}, Kd_l = {self.Kd_l:2.2f}")
        plt.legend()
        ax1.grid(True)
        
        # plot right
        ax2 = plt.subplot(2, 1, 2)
        plt.plot(self.time_list, self.right_speed_list, label='Right Wheel Speed')
        plt.plot(self.time_list, self.desired_right_speed_list, label='desired right')
        plt.xlabel('Time (s)')
        plt.ylabel('Right Wheel Speed (rad/s)')
        plt.title(f"Right Motor Controller, Kp_r = {self.Kp_r:2.2f}, Ki_r = {self.Ki_r:2.2f}, Kd_r = {self.Kd_r:2.2f}")
        plt.legend()
        ax2.grid(True)

        # Optionally adjust layout
        plt.tight_layout()

        # Save the figure
        plt.savefig(f'{self.save_path}motor_controller_{self.formatted_time}.png', format='png', dpi=300)

    def data(self):
        with open(f'{self.save_path}motor_controller_{self.formatted_time}.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'Left Wheel Speed (rad/s)', 'Right Wheel Speed (rad/s)', 'Desired Left Speed (rad/s)', 'Desired Right Speed (rad/s)'])
            for i in range(len(self.time_list)):
                writer.writerow([self.time_list[i], self.left_speed_list[i], self.right_speed_list[i], self.desired_left_speed_list[i], self.desired_right_speed_list[i]])

    def get_save_path(self):
        rospack = rospkg.RosPack()
        return rospack.get_path("controller") + '/src/data/'

    @staticmethod
    def saturation(value, limit):
        value = limit if value > limit else value
        value = -limit if value < -limit else value
        return value

if __name__ == '__main__':
    controller = MotorController()
    rospy.spin()
