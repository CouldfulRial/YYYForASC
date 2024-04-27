#!/usr/bin/env python

'''
This node controls the motion (outer controller)
Subscribed topics:
    odom                      [nav_msgs/Odometry]
    ref_pose                  [geometry_msgs/Pose2D]
Published topics:
    reference_wheel_speeds    [asclinic_pkg/asclinc_pkg]
'''

import rospy
from math import pi, sin, cos, sqrt, atan2
import tf.transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Twist, Vector3
from asclinic_pkg.msg  import LeftRightFloat32
import matplotlib.pyplot as plt

# Constants
WHEEL_DIAMETER = 0.144  # m
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_BASE = 0.218  # m
HALF_WHEEL_BASE = WHEEL_BASE / 2
SPEED_LIMIT = 0.1

EPSILLON = 0.1

class MotionController:
    def __init__(self):
        # Get user parameter
        self.verbosity       = 1
        self.plot            = 1
        self.controller_type = 2
        rospy.init_node('motion_controller', anonymous=True)

        # Subscriber to the Odometry messages
        # odom is publishing at 10Hz
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        # self.pos_sub = rospy.Subscriber('ref_pose', Pose2D, self.pose_callback)

        # Timer: Calls the timer_callback function at 10 Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # Publisher for the reference wheel speeds
        self.speed_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Desired pose for testing
        self.desired_pose = Pose2D(x=5, y=0, theta=0)

        # Register the shutdown callback ==> plot
        rospy.on_shutdown(self.shutdown_callback)
        if self.plot == 1:
            self.current_pose_x_list = []
            self.current_pose_y_list = []
            self.current_theta__list = []
            self.desired_pose_x_list = []
            self.desired_pose_y_list = []
            self.desired_theta__list = []
            if self.controller_type == 2:
                self.alpha_list      = []
                self.beta_list       = []
                self.rho_list        = []

        # Initialise positions
        self.current_pose_x = 0
        self.current_pose_y = 0
        self.current_theta  = 0

        # Initialise other variables
        if self.controller_type == 2:
            self.alpha = 0
            self.beta  = 0
            self.rho   = 0

        # Initialise desired pose
        # self.desired_pose = Pose2D(0, 0, 0)


    def timer_callback(self, event):
        # Store the current pose
        if self.plot == 1:
            self.current_pose_x_list.append(self.current_pose_x)
            self.current_pose_y_list.append(self.current_pose_y)
            self.current_theta__list.append(self.current_theta)
            self.desired_pose_x_list.append(self.desired_pose.x)
            self.desired_pose_y_list.append(self.desired_pose.y)
            self.desired_theta__list.append(self.desired_pose.theta)
            if self.controller_type == 2:
                self.alpha_list.append(self.alpha)
                self.beta_list.append(self.beta)
                self.rho_list.append(self.rho)

        # Compute pose error
        error_x     = self.desired_pose.x     - self.current_pose_x
        error_y     = self.desired_pose.y     - self.current_pose_y
        # error_theta = self.desired_pose.theta - self.current_theta
        error_theta = self.get_error_theta(self.desired_pose.theta, self.current_theta)

        # Compute control signals
        # Simple proportional controller for demonstration
        # controller = self.controller1 if self.controller_type == 1 else self.controller2  # Choose controller
        controller = self.controller2
        v, omega = controller(error_x, error_y, error_theta, self.current_theta)

        # Display log
        if self.verbosity == 1:
            rospy.loginfo("-"*25 + "Motion Controller" + "-"*25 + 
                        f"\nUsing {controller.__name__}" +
                        f"\ndesired_pose_x: {self.desired_pose.x:3.2f}, desired_pose_y: {self.desired_pose.y:3.2f}, desired_theta: {self.desired_pose.theta:3.2f}" + 
                        f"\ncurrent_pose_x: {self.current_pose_x:3.2f}, current_pose_y: {self.current_pose_y:3.2f}, current_theta: {self.current_theta:3.2f}" + 
                        f"\nv:              {v:3.2f}, omega: {omega:3.2f}")

        # Publish the calculated wheel speeds
        self.speed_pub.publish(Twist(linear=Vector3(v, 0, 0), angular=Vector3(0, 0, omega)))

    def odom_callback(self, data):
        # Extract the current pose from the odometry message
        self.current_pose_x = data.pose.pose.position.x
        self.current_pose_y = data.pose.pose.position.y
        self.current_theta  = data.pose.pose.orientation
        self.current_theta = self.quat_to_euler(self.current_theta)

    def pose_callback(self, data):
        self.desired_pose = data

    @staticmethod
    def quat_to_euler(quat):
        if quat is not None:
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            return euler[2]  # return the yaw, which is the orientation around z-axis
        else:
            return 0  # If intialised to None, return 0

    def controller1(self, ex, ey, etheta, theta):
        '''
        This is a simple implementation of a P(I) controller
        '''
        # Controller parameters
        # Gains
        # Position
        Kp_pos = 0.05

        # Angular
        Kp_ang = 0.05

        # Initialise controller integrals
        integral_ex_robot = 0
        integral_etheta   = 0

        self.ex_robot = ex * cos(theta) + ey * sin(theta)

        # I
        integral_ex_robot += self.ex_robot
        integral_etheta += etheta

        v     = Kp_pos * self.ex_robot
        omega = Kp_ang * etheta       

        return v, omega
    
    def controller2(self, x, y, yaw, tx, ty, tyaw,):
        '''
        This implementation is based on the website:
        https://www.bilibili.com/video/BV19C4y1U7TE/?share_source=copy_web&vd_source=53bbd60e60dc232b7e76c75b2d1024c5
        '''
        # Compute pose error
        ex     = tx - x
        ey     = ty - y
        etheta = self.get_error_theta(tyaw, yaw)

        # Controller parameters
        # Gains
        k_rho   = 0.05
        k_theta  = 0.05 #if self.rho < EPSILLON else 0
        # adjusting alpha is useless if too close to the goal
        k_alpha = 0.5 if self.rho > EPSILLON else 0

        # Map from global to robot frame in polar coordinates
        # The linear distance to the goal
        self.rho = sqrt(ex**2 + ey**2)  
        # The angle between the goal theta and the current position of the robot
        self.beta = atan2(ey, ex)
        # Intended angle between the robot and the direction of rho
        self.alpha = self.get_error_theta(self.beta, yaw)

        # Orientation considerations
        # NOTE that to take this into account, we need the measured angle between -pi and pi, i.e. the standard odometry return
        if self.alpha > -pi / 2 and self.alpha < pi / 2:
            # We define that the robot is facing the goal
            v = k_rho * self.rho
            omega = k_alpha * self.alpha + k_theta * etheta
        else:
            # The robot is facing the opposite direction
            v = -k_rho * self.rho
            omega = k_alpha * self.alpha + k_theta * etheta

        return v, omega
            
    def choose_controller(self, desired_pose:Pose2D):
        # Controller is dependent of the command
        if desired_pose == Pose2D(1, 0, 0) or desired_pose == Pose2D(0, 0, pi):
            controller = self.controller2
        else:
            controller = self.controller1
        
        return controller
    
    def shutdown_callback(self):
        if self.plot == 1:
            self.plot_callback()

    def plot_callback(self):
        plt.rcParams.update({'font.size': 5})

        ax1 = plt.subplot(2, 2, 1)
        plt.plot(self.current_pose_x_list, label="current_x")
        plt.plot(self.current_pose_y_list, label="current_y")
        plt.plot(self.desired_pose_x_list, label="desired_x")
        plt.plot(self.desired_pose_y_list, label="desired_y")
        if self.controller_type == 2:
            plt.plot(self.rho_list, label="rho")
        plt.xlabel('steps')
        plt.ylabel('pose_linear')
        # plt.title("Motion Controller")
        plt.legend()
        ax1.grid(True)

        ax2 = plt.subplot(2, 2, 3)
        plt.plot(self.current_theta__list, label="current_theta")
        plt.plot(self.desired_theta__list, label="desired_theta")
        if self.controller_type == 2:
            plt.plot(self.alpha_list, label="alpha")
            plt.plot(self.beta_list, label="beta")
        plt.xlabel('steps')
        plt.ylabel('pose_angle')
        plt.yticks([-pi, -pi/2, 0, pi/2, pi], ['-pi', '-pi/2', '0', 'pi/2', 'pi'])
        plt.legend()
        ax2.grid(True)

        ax3 = plt.subplot(2, 2, 4)
        plt.plot(self.current_pose_x_list, self.current_pose_y_list, label="robot_trajectory")
        plt.plot(self.desired_pose_x_list, self.desired_pose_y_list, label="planned_trajectory")
        plt.xlabel('x')
        plt.ylabel('y')
        # plt.title("Trajectories")
        plt.legend()
         # Set the aspect of the plot to 'equal' to make the plot square
        ax3.set_aspect('equal', adjustable='box')
        # Add grid to the subplot
        ax3.grid(True)

        # plt.show()
        # Optionally adjust layout
        plt.tight_layout()

        # Save the figure
        plt.savefig('/home/asc/YYYForASC/catkin_ws/src/navigation/src/data/plot.png', format='png', dpi=300)

    @staticmethod
    def get_error_theta(desired_theta, current_theta):
        # for any theta, it is the same if -2pi or 2pi
        # We need two reference thetas, one in the positive and one in the negative direction
        desired_theta_neg = desired_theta - 2 * pi if desired_theta > 0 else desired_theta
        desired_theta_pos = desired_theta + 2 * pi if desired_theta < 0 else desired_theta
        
        # Always calculate the error based on the current theta negativity
        error_theta = desired_theta_neg - current_theta if current_theta < 0 else desired_theta_pos - current_theta

        return error_theta


if __name__ == '__main__':
    try:
        controller = MotionController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
