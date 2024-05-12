#!/usr/bin/env python

'''
This node implements the pure pursuit controller
Subscribed topics:
    path    [nav_msgs/Path]
    odom    [nav_msgs/Odometry]
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
import matplotlib.pyplot as plt
import numpy as np

# Parmeters
TIME_STEP = 0.1  # s
PATH = [
    np.array([0.0000, 0.0000]),
    np.array([0.0591, 0.0807]),
    np.array([0.1256, 0.1553]),
    np.array([0.2088, 0.2109]),
    np.array([0.2914, 0.2672]),
    np.array([0.3318, 0.3587]),
    np.array([0.4201, 0.4057]),
    np.array([0.5021, 0.4628]),
    np.array([0.5655, 0.5402]),
    np.array([0.6557, 0.5834]),
    np.array([0.7238, 0.6566]),
    np.array([0.8079, 0.7107]),
    np.array([0.8882, 0.7702]),
    np.array([0.9627, 0.8369]),
    np.array([1.0029, 0.9285]),
    np.array([1.0641, 1.0076]),
    np.array([1.1051, 1.0988]),
    np.array([1.1850, 1.1588]),
    np.array([1.2472, 1.2372]),
    np.array([1.2893, 1.3279]),
    np.array([1.3540, 1.4041]),
    np.array([1.4111, 1.4862]),
    np.array([1.4886, 1.5495]),
    np.array([1.5356, 1.6377]),
    np.array([1.6057, 1.7091]),
    np.array([1.6823, 1.7733]),
    np.array([1.7764, 1.8073]),
    np.array([1.8589, 1.8639]),
    np.array([1.9433, 1.9175]),
    np.array([2.0141, 1.9880]),
    np.array([2.0000, 2.0000])
]

class TrajectoryTracker:
    def __init__(self):
        # Initialise node
        self.node_name = 'trajectory_tracker'
        rospy.init_node(self.node_name, anonymous=True)

        # Get user parameter
        try:
            self.parm  = rospy.get_param(self.node_name)
            self.verbosity = self.parm["verbosity"]
            self.save_data = self.parm["save_data"]
            self.simulate = self.parm["simulate"]
        except KeyError:
            self.verbosity = 1
            self.save_data = 1
            self.simulate = 0

        # Initialise the variables
        self.acceleration, self.omega = 0.0, 0.0  # The command input to the robot
        self.target_position = np.zeros((2, 1))  # The current position to track
        self.old_target_pos = np.zeros((2, 1))  # The previous position to track
        self.target_velocity = np.zeros((2, 1))  # The current velocity to track
        self.position = np.zeros((2, 1))  # The current position of the robot
        self.yaw = 0.0  # The current orientation of the robot
        self.twist = np.zeros((2, 1))  # The current twist of the robot
        self.velocity = np.zeros((2, 1))  # The current inertial velocity of the robot
        self.initial_time = rospy.Time.now().to_sec()  # The initial time
        self.current_time = 0.0  # The current time

        # Subscribers
        self.path_subscriber = rospy.Subscriber('path', Path, self.path_callback)
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
        pass

    def odom_callback(self, data:Odometry):
        # Get the current position
        self.position = np.array([[data.pose.pose.position.x], [data.pose.pose.position.y]])
        self.yaw = self.quat_to_euler(data.pose.pose.orientation)

        # Get the current twist
        self.twist = np.array([[data.twist.twist.linear.x], [data.twist.twist.angular.z]])

        # Map to the inertial velocity
        self.velocity = np.array([[self.twist[0] * np.cos(self.yaw)], [self.twist[0] * np.sin(self.yaw)]])

    def timer_callback(self, event):
        # Execute the control loop
        v, omega = self.simple_PD_controller()
        # v = np.clip(v, -0.3, 0.4)
        # omega = np.clip(omega, -0.4, 0.4)

        # Logging
        if self.verbosity == 1:
            x = self.position[0, 0]
            y = self.position[1, 0]
            tx = self.target_position[0, 0]
            ty = self.target_position[1, 0]
            cv = self.twist[0, 0]
            comega = self.twist[1, 0]
            print("target velocity: \n", self.target_velocity)
            print("acceleration: \n", self.acceleration)
            print("velocity: \n", self.velocity)
            rospy.loginfo("-"*25 + self.node_name + "-"*25 +
                        f"\nx: {x:3.5f}, y: {y:3.5f}, yaw: {self.yaw/np.pi:3.5f}pi" +
                        f"\ntarget: ({tx:3.5f}, {ty:3.5f})" + 
                        f"\nv: {cv:3.5f}, omega: {comega:3.5f}" +
                        f"\ncmd_v: {v:3.5f}, cmd_omega: {omega:3.5f}" + 
                        "\n")

        # Data saving
        if self.save_data == 1:
            self.time_list.append(self.current_time)
            self.x_list.append(self.position[0, 0])
            self.y_list.append(self.position[1, 0])
            self.yaw_list.append(self.yaw)
            self.target_x_list.append(self.target_position[0, 0])
            self.target_y_list.append(self.target_position[1, 0])
            # self.target_yaw_list.append(self.target_yaw)
        
        # Publish the velocity command
        self.cmd_vel_publisher.publish(
            Twist(
                Vector3(v, 0, 0),
                Vector3(0, 0, omega)
            )
        )

    def simple_PD_controller(self):
        # https://www.youtube.com/watch?v=OcsD-1wINK0&t=313s
        # A simple PD controller for trajectory tracking

        # Get the current tracking position, increment by second
        self.current_time = rospy.Time.now().to_sec() - self.initial_time
        idx = np.floor(self.current_time) + 1
        idx = int(idx)
        if idx > (len(PATH) - 1):
            # No more point to track
            rospy.signal_shutdown("End of Path")
            return 0.0, 0.0
        else:
            self.target_position = PATH[idx].reshape(-1, 1)  # Use reshape instead of T because it is a 1D array
            self.old_target_pos = PATH[idx - 1].reshape(-1, 1)

        # Keep track of target position and velocity
        self.target_velocity = (self.target_position - self.old_target_pos)

        # A PD controller to get desired velocity
        Kp = 0.5
        Kd = 0.5
        position_error = self.target_position - self.position
        velocity_error = self.target_velocity - self.velocity
        self.acceleration = Kp * position_error + Kd * velocity_error;  # a_x; a_y

        # Map the desired acceleration to the velocity and steering
        v_unit_vec = np.array([np.cos(self.yaw), np.sin(self.yaw)])  # 1x2
        acceleration = v_unit_vec @ self.acceleration
        v = self.velocity[0] + acceleration

        omega_unit_vec = np.array([np.cos(self.yaw + np.pi/2), np.sin(self.yaw + np.pi/2)])  # 1x2
        omega = omega_unit_vec @ self.acceleration

        return float(v[0]), float(omega[0])


    ##############################################################################################################
    ############################## Untilities ####################################################################
    ##############################################################################################################
    
    @staticmethod
    def wrap_angle(angle):
        # Angle wrapping
        return (angle + np.pi) % (2 * np.pi) - np.pi

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
        self.cmd_vel_publisher.publish(
            Twist(
                Vector3(0.0, 0, 0),
                Vector3(0, 0, 0.0)
            )
        )
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
        TrajectoryTracker.plot_vectors(self.x_list, self.y_list, self.yaw_list)

        # Format the plot
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.legend()
        plt.title(f"Position Plot")
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
        plt.title(f"Trakcing Plot")
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
        # plt.plot(self.time_list, self.target_yaw_list, label='Yaw Target')
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw (rad)')
        plt.legend()
        ax3.grid(True)

        # Save the figure
        plt.savefig(f'{self.save_path}motion_{self.formatted_time}.png', format='png', dpi=300)

    @staticmethod
    def plot_vectors(x_list, y_list, theta_radians, length=0.1, num=10):
        # Truncate
        x_list = TrajectoryTracker.slice_evenly(x_list, num)
        y_list = TrajectoryTracker.slice_evenly(y_list, num)
        theta_radians = TrajectoryTracker.slice_evenly(theta_radians, num)
        
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
    controller = TrajectoryTracker()
    rospy.spin()
