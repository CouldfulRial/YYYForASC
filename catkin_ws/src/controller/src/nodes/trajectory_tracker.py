#!/usr/bin/env python

'''
This node implements the pure pursuit controller
Subscribed topics:
    path    [nav_msgs/Path]
    odom   [nav_msgs/Odometry]
Published topics:
    cmd_vel [geometry_msgs/Twist]
'''
import rospy
import rospkg
import datetime
import csv
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Vector3, Pose2D
import tf
import math
from math import pi
import matplotlib.pyplot as plt
import numpy as np

# Parmeters
TIME_STEP = 0.1  # s

class SimpleMotionController:
    def __init__(self):
        # Initialise node
        self.node_name = 'trajectory_tracker'
        rospy.init_node(self.node_name, anonymous=True)

        # Get user parameter
        self.parm  = rospy.get_param(self.node_name)
        self.verbosity = self.parm["verbosity"]
        self.save_data = self.parm["save_data"]
        self.simulate = self.parm["simulate"]

        # Initialise the variables
        self.current_path = None
        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0

        # Subscribers
        self.path_subscriber = rospy.Subscriber('ref_pose', Pose2D, self.path_callback)
        self.odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Timer: Calls the timer_callback function at 10 Hz
        self.timer = rospy.Timer(rospy.Duration(TIME_STEP), self.timer_callback)

        # Publisher
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Register the shutdown callback
        rospy.on_shutdown(self.shutdown_callback)

        # Intialise data saving
        if self.save_data == 1:
            self.time_list = [0]
            self.x_list = [0]
            self.y_list = [0]
            self.yaw_list = [0]
            self.target_x_list = [0]
            self.target_y_list = [0]
            self.target_yaw_list = [0]
            # Get save path. Set the file name to the intial time
            self.save_path = self.get_save_path()
            intial_time = datetime.datetime.now()
            self.formatted_time = intial_time.strftime('%y%m%d_%H_%M_%S')

    def path_callback(self, data):
        self.target_x = data.x
        self.target_y = data.y
        self.target_yaw = data.theta

    def odom_callback(self, data:Odometry):
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        self.current_yaw = self.quat_to_euler(data.pose.pose.orientation)

    def timer_callback(self, event):
        # Execute the control loop
        v, omega, rho, alpha, beta, self.P_rho, self.P_alpha, self.P_theta = self.simple_controller(
            self.current_x, self.current_y, self.current_yaw,
            self.target_x,  self.target_y,  self.target_yaw)

        # Logging
        if self.verbosity == 1:
            rospy.loginfo("-"*25 + "Simple Motion Controller" + "-"*25 +
                        f"\nx: {self.current_x:3.5f}, y: {self.current_y:3.5f}, yaw: {self.current_yaw/pi:3.5f}pi" +
                        f"\nalpha: {alpha/pi:3.5f}pi, beta: {beta/pi:3.5f}pi, rho: {rho:3.5f}" +
                        f"\nv: {v:3.5f}, omega: {omega:3.5f}"
                        f"\ntarget: ({self.target_x:3.5f}, {self.target_y:3.5f}, {self.target_yaw/pi:3.5f}pi)" + 
                        "\n")

        # Data saving
        if self.save_data == 1:
            self.time_list.append(self.time_list[-1] + TIME_STEP)
            self.x_list.append(self.current_x)
            self.y_list.append(self.current_y)
            self.yaw_list.append(self.current_yaw)
            self.target_x_list.append(self.target_x)
            self.target_y_list.append(self.target_y)
            self.target_yaw_list.append(self.target_yaw)
        
        # Publish the velocity command
        self.cmd_vel_publisher.publish(
            Twist(
                Vector3(v, 0, 0),
                Vector3(0, 0, omega)
            )
        )

    @staticmethod
    def simple_controller(x, y, yaw, tx, ty, tyaw):
        # https://www.bilibili.com/video/BV19C4y1U7TE/?share_source=copy_web&vd_source=53bbd60e60dc232b7e76c75b2d1024c5
        # Compute pose error
        ex     = tx - x
        ey     = ty - y
        etheta = SimpleMotionController.wrap_angle(tyaw - yaw)

        # Map from global to robot frame in polar coordinates
        # The linear distance to the goal
        rho = math.sqrt(ex**2 + ey**2)  
        # The angle between the goal theta and the current position of the robot
        beta = math.atan2(ey, ex)
        # Intended angle between the robot and the direction of rho
        alpha = SimpleMotionController.wrap_angle(beta - yaw)

        # Controller parameters
        # Gains
        P_rho   = 0.5 if rho > 0.1 else 0.01
        P_theta  = 1 if rho < 0.1 else 0
        # adjusting alpha is useless if too close to the goal
        P_alpha = 2 if rho > 0.1 else 0

        # Orientation considerations
        # NOTE that to take this into account, we need the measured angle between -pi and pi, i.e. the standard odometry return
        # v = P_rho * rho
        # omega = P_alpha * alpha + P_theta * etheta
        if alpha > -pi / 2 and alpha < pi / 2:
            # We define that the robot is facing the goal
            v = P_rho * rho
            omega = P_alpha * alpha + P_theta * etheta
        else:
            # The robot is facing the opposite direction
            v = -P_rho * rho
            sign_alpha = 1 if alpha < 0 else -1
            omega = P_alpha * sign_alpha * (pi - abs(alpha)) + P_theta * etheta
        
        v     = np.clip(v    , -0.3, 0.4)
        omega = np.clip(omega, -0.7, 0.7)
        return v, omega, rho, alpha, beta, P_rho, P_alpha, P_theta

    ##############################################################################################################
    ############################## Untilities ####################################################################
    ##############################################################################################################
    
    @staticmethod
    def wrap_angle(angle):
        # Angle wrapping
        return (angle + pi) % (2 * pi) - pi

    @staticmethod
    def quat_to_euler(quat):
        if quat is not None:
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            return euler[2]  # return the yaw, which is the orientation around z-axis
        else:
            return 0  # If intialised to None, return 0

    ##############################################################################################################
    ############################## The following codes are not relevant to algorithm ##############################
    ##############################################################################################################
    def shutdown_callback(self):
        rospy.loginfo(f"Shutting down node {self.node_name}....")
        if self.save_data == 1:
            rospy.loginfo(f"Saving Data To {self.save_path}....")
            self.plot()
            self.plot_time()
            self.data()
            rospy.loginfo("Saving Data Completed....")

    def plot(self):
        # set the font size
        plt.rcParams.update({'font.size': 5})

        # plot
        plt.plot(self.x_list, self.y_list, label='Robot Path')
        plt.plot(self.target_x_list, self.target_y_list, label='Planned Path')
        # Plot the directions
        SimpleMotionController.plot_vectors(self.x_list, self.y_list, self.yaw_list)

        # Format the plot
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.legend()
        plt.title(f"Position Plot\n P_rho={self.P_rho}, P_alpha={self.P_alpha}, P_theta={self.P_theta}")
        plt.grid(True)
        plt.axis('equal')

        # Save the figure
        plt.savefig(f'{self.save_path}pure_pursuit_controller_{self.formatted_time}.png', format='png', dpi=300)

    def plot_time(self):
        # set the font size
        plt.rcParams.update({'font.size': 5})

        # plot
        ax1 = plt.subplot(3, 1, 1)
        plt.plot(self.time_list, self.x_list, label='X Position')
        plt.plot(self.time_list, self.target_x_list, label='X Target')
        plt.ylabel('X Position (m)')
        plt.title(f"Trakcing Plot\n P_rho={self.P_rho}, P_alpha={self.P_alpha}, P_theta={self.P_theta}")
        plt.legend()
        ax1.grid(True)

        ax2 = plt.subplot(3, 1, 2)
        plt.plot(self.time_list, self.y_list, label='Y Position')
        plt.plot(self.time_list, self.target_y_list, label='Y Target')
        plt.ylabel('Y Position (m)')
        plt.legend()
        ax2.grid(True)

        ax3 = plt.subplot(3, 1, 3)
        plt.plot(self.time_list, self.yaw_list, label='Yaw Position')
        plt.plot(self.time_list, self.target_yaw_list, label='Yaw Target')
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw (rad)')
        plt.legend()
        ax3.grid(True)

        # Save the figure
        plt.savefig(f'{self.save_path}motion_{self.formatted_time}.png', format='png', dpi=300)

    @staticmethod
    def plot_vectors(x_list, y_list, theta_radians, length=0.1, num=10):
        # Truncate
        x_list = SimpleMotionController.slice_evenly(x_list, num)
        y_list = SimpleMotionController.slice_evenly(y_list, num)
        theta_radians = SimpleMotionController.slice_evenly(theta_radians, num)
        
        # Calculate the components of the vectors
        dx = length * np.cos(theta_radians)
        dy = length * np.sin(theta_radians)

        # Plot the vectors
        plt.quiver(x_list, y_list, dx, dy, angles='xy', scale_units='xy', scale=1)

    @staticmethod
    def slice_evenly(original_list, number_of_elements):
        length = len(original_list)
        print("length: ", length)
        step = length // number_of_elements
        # sliced_list = 
        return original_list[0::step]
        #  sliced_list[:number_of_elements]

    def data(self):
        with open(f'{self.save_path}pure_pursuit_controller_{self.formatted_time}.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'X Position (m)', 'Y Position (m)', "Yaw (rad)"])
            for i in range(len(self.time_list)):
                writer.writerow([self.time_list[i], self.x_list[i], self.y_list[i], self.yaw_list[i]])

    @staticmethod
    def get_save_path():
        rospack = rospkg.RosPack()
        return rospack.get_path("controller") + '/src/data/'

if __name__ == '__main__':
    controller = SimpleMotionController()
    rospy.spin()
