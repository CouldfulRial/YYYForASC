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
from geometry_msgs.msg import Twist, Vector3
import math
import tf
import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np

# Parmeters
DEBUG_LENGTH = 10
TIME_STEP = 0.1  # s
LOOKAHEAD = 0.2
WB = 0.04
FREQS = 10

class PurePursuitController:
    def __init__(self):
        # Initialise node
        self.node_name = 'pure_pursuit_controller'
        rospy.init_node(self.node_name, anonymous=True)

        # Get user parameter
        # self.parm  = rospy.get_param(self.node_name)
        # self.verbosity = self.parm["verbosity"]
        # self.save_data = self.parm["save_data"]
        # self.debug = self.parm["debug"]  # If debug, only run for DEBUG_LENGTH seconds

        # Subscribers
        # self.path_subscriber = rospy.Subscriber('path', Path, self.path_callback)
        self.odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Publisher
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Register the shutdown callback
        rospy.on_shutdown(self.shutdown_callback)

        # Intialise data saving
        self.time_list = [0]
        self.x_list = [0]
        self.y_list = [0]
        # Get save path. Set the file name to the intial time
        self.save_path = self.get_save_path()
        intial_time = datetime.datetime.now()
        self.formatted_time = intial_time.strftime('%y%m%d_%H_%M_%S')

        self.v_prev_error = 0.0

    def path_callback(self, msg):
        self.current_path = msg

    def odom_callback(self, data:Odometry):
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        self.current_yaw = self.quat_to_euler(data.pose.pose.orientation)
        self.current_vel = np.linalg.norm(np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]),2)

        # Logging
        rospy.loginfo("-"*25 + "Pure Pursuit Controller" + "-"*25 +
                        f"\nx: {self.current_x:3.5f}, y: {self.current_y:3.5f}, yaw: {self.current_yaw:3.5f}")

        # Data saving
        self.time_list.append(self.time_list[-1] + TIME_STEP)
        self.x_list.append(self.current_x)
        self.y_list.append(self.current_y)

        # Execute the control loop
        waypoints = [(0, 0)]
        v, omega = self.pure_pursuit_control(self.current_x, self.current_y, self.current_yaw,
                                             waypoints, 0.5, 0.5, 0.1)
        
        # Publish the velocity command
        self.cmd_vel_publisher.publish(
            Twist(
                Vector3(v, 0, 0),
                Vector3(0, 0, omega)
            )
        )

    @staticmethod
    def pure_pursuit_control(x, y, theta, waypoints, lookahead_distance, max_velocity, stopping_threshold):
        # Calculate distance to the final waypoint
        final_waypoint = np.array(waypoints[-1])
        distance_to_final = np.linalg.norm(final_waypoint - np.array([x, y]))

        # Stop if close to the final waypoint
        if distance_to_final <= stopping_threshold:
            return 0, 0  # Stop the robot by setting velocities to zero
        
        # Find the target point in the lookahead distance
        target_point = PurePursuitController.find_target_point(x, y, waypoints, lookahead_distance)
        
        # Calculate the angle to the target point
        target_angle = np.arctan2(target_point[1] - y, target_point[0] - x)
        angle_difference = target_angle - theta
        angle_difference = (angle_difference + np.pi) % (2 * np.pi) - np.pi  # Normalize angle to [-pi, pi]

        # Control law for angular velocity
        omega = 2 * max_velocity * np.sin(angle_difference) / lookahead_distance
        
        # Constant velocity assumption
        v = max_velocity

        return v, omega
    
    @staticmethod
    def find_target_point(x, y, waypoints, L):
        for i in range(len(waypoints) - 1):
            # Line segment from current waypoint to next waypoint
            start = np.array(waypoints[i])
            end = np.array(waypoints[i + 1])

            # Vector from robot to start and end of segment
            start_vector = start - np.array([x, y])
            end_vector = end - np.array([x, y])

            # Projection factor
            t = np.dot(end_vector - start_vector, start_vector) / np.linalg.norm(end_vector - start_vector)**2
            t = np.clip(t, 0, 1)

            # Closest point to robot on segment
            closest_point = start + t * (end_vector - start_vector)
            distance = np.linalg.norm(closest_point - np.array([x, y]))

            # Check if the closest point is within the lookahead distance
            if distance <= L:
                return closest_point
            
        return waypoints[-1]  # Return the last waypoint if none are within L

    def control_loop(self):
        if not self.current_path or not self.current_pose:
            return  # Do nothing if no path or pose yet

        # Find the closest path point ahead of the robot by at least the look-ahead distance
        closest_point = None
        min_distance = float('inf')

        for pose in self.current_path.poses:
            # Calculate the distance between the robot and the path point
            dx = pose.pose.position.x - self.current_pose.position.x
            dy = pose.pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)

            # Update the closest point if needed
            if distance < min_distance and distance > self.look_ahead_distance:
                min_distance = distance
                closest_point = pose.pose

        if closest_point is None:
            rospy.loginfo("No valid look-ahead point found.")
            return

        # Calculate the steering angle needed to reach the look-ahead point
        angle_to_point = math.atan2(closest_point.position.y - self.current_pose.position.y,
                                    closest_point.position.x - self.current_pose.position.x)

        # Calculate robot's current yaw from quaternion
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        # Steering command
        angle_diff = angle_to_point - yaw
        curvature = 2 * math.sin(angle_diff) / self.look_ahead_distance
        velocity_command = Twist()

        # Simple proportional controller for speed (can be replaced with a more sophisticated controller)
        velocity_command.linear.x = 0.5 * max(0, 1 - abs(angle_diff))  # Slow down on sharp turns
        velocity_command.angular.z = curvature

        # Logging
        rospy.loginfo("-"*25 + "Pure Pursuit Controller" + "-"*25 + 
                     f"\nx: {self.current_pose.position.x:3.5f}, y: {self.current_pose.position.y:3.5f}" + 
                     f"\nyaw: {yaw:3.5f}, angle_to_point: {angle_to_point:3.5f}, curvature: {curvature:3.5f}" +
                     f"\nclosest_point: ({closest_point.position.x:3.5f}, {closest_point.position.y:3.5f})")
        
        # Data saving
        self.time_list.append(self.time_list[-1] + TIME_STEP)
        self.x_list.append(self.current_pose.position.x)
        self.y_list.append(self.current_pose.position.y)

        # Publish the velocity command
        self.cmd_vel_publisher.publish(velocity_command)

    ##############################################################################################################
    ############################## Untilities ####################################################################
    ##############################################################################################################
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
        rospy.loginfo(f"Saving Data To {self.save_path}....")
        self.plot()
        self.data()
        rospy.loginfo("Saving Data Completed....")

    def plot(self):
        # set the font size
        plt.rcParams.update({'font.size': 5})
        
        # plot right
        plt.plot(self.x_list, self.y_list, label='Robot Path')
        # plt.plot(self.time_list, self.y_list, label='Planned Path')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')

        # Save the figure
        plt.savefig(f'{self.save_path}pure_pursuit_controller_{self.formatted_time}.png', format='png', dpi=300)

    def data(self):
        with open(f'{self.save_path}pure_pursuit_controller_{self.formatted_time}.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'X Position (m)', 'Y Position (m)'])
            for i in range(len(self.time_list)):
                writer.writerow([self.time_list[i], self.x_list[i], self.y_list[i]])

    def get_save_path(self):
        rospack = rospkg.RosPack()
        return rospack.get_path("controller") + '/src/data/'

if __name__ == '__main__':
    controller = PurePursuitController()
    rospy.spin()
