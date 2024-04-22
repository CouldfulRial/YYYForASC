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
from math import pi, sin, cos
import tf.transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from asclinic_pkg.msg  import LeftRightFloat32
import matplotlib.pyplot as plt

# Constants
WHEEL_DIAMETER = 0.144  # m
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_BASE = 0.218  # m
HALF_WHEEL_BASE = WHEEL_BASE / 2

EPSILON = 0.0000001

class MotionController:
    def __init__(self):
        # Get user parameter
        self.verbosity = rospy.get_param('~verbosity', 0)
        self.plot = rospy.get_param('~plot', 1)

        rospy.init_node('motion_controller', anonymous=True)

        # Subscriber to the Odometry messages
        # odom is publishing at 10Hz
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.pos_sub = rospy.Subscriber('ref_pose', Pose2D, self.pose_callback)

        # Timer: Calls the timer_callback function at 10 Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # Publisher for the reference wheel speeds
        self.speed_pub = rospy.Publisher('reference_wheel_speeds', LeftRightFloat32, queue_size=10)

        # Desired pose, later will be topic
        # self.desired_pose = Pose2D(x=0, y=0, theta=3*pi/2)

        # Register the shutdown callback ==> plot
        rospy.on_shutdown(self.shutdown_callback)
        if self.plot == 1:
            self.current_pose_x_list = []
            self.current_pose_y_list = []
            self.current_theta__list = []
            self.desired_pose_x_list = []
            self.desired_pose_y_list = []
            self.desired_theta__list = []

        # Initialise positions
        self.current_pose_x = 0
        self.current_pose_y = 0
        self.current_theta  = 0

        # Initialise desired pose
        self.desired_pose = Pose2D(0, 0, 0)

        # Controller gains
        # Position
        self.Kp_pos = 0.03
        self.Ki_pos = 0

        # Angular
        self.Kp_ang = 0.03
        self.Ki_ang = 0

        # Initialise controller integrals
        self.integral_ex_robot = 0
        self.integral_ey_robot = 0
        self.integral_etheta = 0

    def timer_callback(self, event):
        # Store the current pose
        if self.plot == 1:
            self.current_pose_x_list.append(self.current_pose_x)
            self.current_pose_y_list.append(self.current_pose_y)
            self.current_theta__list.append(self.current_theta)
            self.desired_pose_x_list.append(self.desired_pose.x)
            self.desired_pose_y_list.append(self.desired_pose.y)
            self.desired_theta__list.append(self.desired_pose.theta)

        # Compute pose error
        error_x = self.desired_pose.x - self.current_pose_x
        error_y = self.desired_pose.y - self.current_pose_y
        error_theta = self.desired_pose.theta - self.current_theta

        # Compute control signals
        # Simple proportional controller for demonstration
        v, omega = self.control_Twist(error_x, error_y, error_theta, self.current_theta)

        # Assuming a differential drive robot
        # Convert from linear and angular velocity to wheel speeds
        left_speed = (v - omega * HALF_WHEEL_BASE) / (WHEEL_RADIUS)
        right_speed = (v + omega * HALF_WHEEL_BASE) / (WHEEL_RADIUS)

        # Add saturation to both wheels
        SPEED_LIMIT = 0.25
        if left_speed > SPEED_LIMIT:
            left_speed = SPEED_LIMIT
        elif left_speed < -SPEED_LIMIT:
            left_speed = -SPEED_LIMIT
        if right_speed > SPEED_LIMIT:
            right_speed = SPEED_LIMIT
        elif right_speed < -SPEED_LIMIT:
            right_speed = -SPEED_LIMIT

        # Display log
        if self.verbosity == 1:
            rospy.loginfo("-"*25 + "Motion Controller" + "-"*25 + 
                        f"\ndesired_pose_x: {self.desired_pose.x:3.2f},    desired_pose_y: {self.desired_pose.y:3.2f}, desired_theta: {self.desired_pose.theta:3.2f}" + 
                        f"\ncurrent_pose_x: {self.current_pose_x:3.2f},    current_pose_y: {self.current_pose_y:3.2f}, current_theta: {self.current_theta:3.2f}" + 
                        f"\nex_robot:       {self.ex_robot:3.2f},          ey_robot: {self.ey_robot:3.2f},             error_theta: {error_theta:3.2f}" +
                        f"\nintegral_ex:    {self.integral_ex_robot:3.2f}, integral_ey: {self.integral_ey_robot:3.2f}, integral_etheta: {self.integral_etheta:3.2f}" + 
                        f"\nv:              {v:3.2f},                      omega: {omega:3.2f}" +
                        f"\nleft_speed:     {left_speed:3.2f},             right_speed: {right_speed:3.2f}")

        # Publish the calculated wheel speeds
        self.speed_pub.publish(LeftRightFloat32(left=left_speed, right=right_speed))

    def odom_callback(self, data):
        # Extract the current pose from the odometry message
        self.current_pose_x = data.pose.pose.position.x
        self.current_pose_y = data.pose.pose.position.y
        self.current_theta = data.pose.pose.orientation.z
        # self.current_theta = self.quat_to_euler(current_orientation)
        # map to [0, 2pi]
        # if self.current_theta < 0:
        #     self.current_theta += 2 * pi

    def pose_callback(self, data):
        self.desired_pose = data

    @staticmethod
    def quat_to_euler(quat):
        if quat is not None:
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            return euler[2]  # return the yaw, which is the orientation around z-axis
        else:
            return 0  # If intialised to None, return 0

    def control_Twist(self, ex, ey, etheta, theta):
        # v     = ex / (2 * cos(theta) + EPSILON) + ey / (2 * sin(theta) + EPSILON)
        # omega = etheta
        self.ex_robot = ex * cos(theta) + ey * sin(theta)
        self.ey_robot = -ex * sin(theta) + ey * cos(theta)

        # I
        self.integral_ex_robot += self.ex_robot
        self.integral_ey_robot += self.ey_robot
        self.integral_etheta += etheta

        v = self.Kp_pos * self.ex_robot + self.Ki_pos * self.integral_ex_robot
        omega = self.Kp_ang * etheta + self.Ki_ang * self.integral_etheta

        return v, omega
    
    def shutdown_callback(self):
        if self.plot == 1:
            self.plot_callback()

    def plot_callback(self):
        rospy.loginfo(f"Plotting")
        plt.plot(self.current_pose_x_list, label="current_x")
        plt.plot(self.current_pose_y_list, label="current_y")
        plt.plot(self.current_theta__list, label="current_theta")
        plt.plot(self.desired_pose_x_list, label="desired_x")
        plt.plot(self.desired_pose_y_list, label="desired_y")
        plt.plot(self.desired_theta__list, label="desired_theta")
        plt.xlabel('steps')
        plt.ylabel('pose')
        plt.title("Motion Controller")
        plt.legend()
        plt.show()

if __name__ == '__main__':
    try:
        controller = MotionController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
