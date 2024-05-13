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
SPEED = 1.0  # m/s
k = 0.1  # look forward gain
Lfc = 1  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 0.218  # [m] wheel base of vehicle
# PATH = [
#     np.array([0.0000, 0.0000]),
#     np.array([0.0591, 0.0807]),
#     np.array([0.1256, 0.1553]),
#     np.array([0.2088, 0.2109]),
#     np.array([0.2914, 0.2672]),
#     np.array([0.3318, 0.3587]),
#     np.array([0.4201, 0.4057]),
#     np.array([0.5021, 0.4628]),
#     np.array([0.5655, 0.5402]),
#     np.array([0.6557, 0.5834]),
#     np.array([0.7238, 0.6566]),
#     np.array([0.8079, 0.7107]),
#     np.array([0.8882, 0.7702]),
#     np.array([0.9627, 0.8369]),
#     np.array([1.0029, 0.9285]),
#     np.array([1.0641, 1.0076]),
#     np.array([1.1051, 1.0988]),
#     np.array([1.1850, 1.1588]),
#     np.array([1.2472, 1.2372]),
#     np.array([1.2893, 1.3279]),
#     np.array([1.3540, 1.4041]),
#     np.array([1.4111, 1.4862]),
#     np.array([1.4886, 1.5495]),
#     np.array([1.5356, 1.6377]),
#     np.array([1.6057, 1.7091]),
#     np.array([1.6823, 1.7733]),
#     np.array([1.7764, 1.8073]),
#     np.array([1.8589, 1.8639]),
#     np.array([1.9433, 1.9175]),
#     np.array([2.0141, 1.9880]),
#     np.array([2.0000, 2.0000])
# ]

PATH = [
    np.array([0.0000, 0.0000]),
    np.array([0.2648, -0.0529]),
    np.array([0.5263, -0.1045]),
    np.array([0.7815, -0.1538]),
    np.array([1.0270, -0.1995]),
    np.array([1.2596, -0.2404]),
    np.array([1.4762, -0.2754]),
    np.array([1.6735, -0.3032]),
    np.array([1.8483, -0.3227]),
    np.array([1.9974, -0.3326]),
    np.array([2.1176, -0.3317]),
    np.array([2.2056, -0.3189]),
    np.array([2.2583, -0.2930]),
    np.array([2.2725, -0.2528]),
    np.array([2.2449, -0.1971]),
    np.array([2.1723, -0.1246]),
    np.array([2.0515, -0.0343]),
    np.array([1.8799, 0.0750]),
    np.array([1.6610, 0.2025]),
    np.array([1.4027, 0.3465]),
    np.array([1.1127, 0.5051]),
    np.array([0.7988, 0.6764]),
    np.array([0.4689, 0.8586]),
    np.array([0.1307, 1.0500]),
    np.array([-0.2079, 1.2486]),
    np.array([-0.5392, 1.4526]),
    np.array([-0.8554, 1.6602]),
    np.array([-1.1486, 1.8695]),
    np.array([-1.4111, 2.0787]),
    np.array([-1.6350, 2.2860]),
    np.array([-1.8125, 2.4896]),
    np.array([-1.9359, 2.6875]),
    np.array([-1.9973, 2.8780]),
    np.array([-1.9891, 3.0593]),
    np.array([-1.9080, 3.2304]),
    np.array([-1.7580, 3.3918]),
    np.array([-1.5438, 3.5443]),
    np.array([-1.2699, 3.6884]),
    np.array([-0.9410, 3.8248]),
    np.array([-0.5615, 3.9541]),
    np.array([-0.1362, 4.0770]),
    np.array([0.3304, 4.1941]),
    np.array([0.8338, 4.3061]),
    np.array([1.3693, 4.4136]),
    np.array([1.9324, 4.5172]),
    np.array([2.5183, 4.6176]),
    np.array([3.1227, 4.7155]),
    np.array([3.7408, 4.8114]),
    np.array([4.3681, 4.9060]),
    np.array([5.0000, 5.0000])
]


class TrajectoryTracker:
    def __init__(self):
        # Initialise node
        self.node_name = 'pure_pursuit_tracker'
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
        self.state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
        self.target_x, self.target_y = 0.0, 0.0
        # self.cx = np.array([point[0] for point in PATH])
        # self.cy = np.array([point[1] for point in PATH])
        self.cx = np.arange(0, 50, 0.1)
        self.cy = np.array([np.sin(ix) * 2 for ix in self.cx])
        # self.cx = np.arange(0, 50, 0.1)
        # self.cy = np.zeros_like(self.cx)
        self.target_ind = 0
        self.current_time = rospy.Time.now().to_sec()

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
        # Update the robot state
        self.state.update(x=data.pose.pose.position.x, 
                          y=data.pose.pose.position.y, 
                          yaw=self.quat_to_euler(data.pose.pose.orientation), 
                          v=data.twist.twist.linear.x, 
                          omega=data.twist.twist.angular.z)

    def timer_callback(self, event):
        # Execute the control loop
        self.pure_pursuit_controller()
        # v = np.clip(v, -1, 1)
        # omega = np.clip(omega, -0.4, 0.4)

        # Logging
        if self.verbosity == 1:
            x = self.state.x
            y = self.state.y
            yaw = self.state.yaw
            tx = self.target_x
            ty = self.target_y
            rospy.loginfo("-"*25 + self.node_name + "-"*25 +
                        f"\nx: {x:3.5f}, y: {y:3.5f}, yaw: {yaw/np.pi:3.5f}pi" +
                        f"\ntarget: ({tx:3.5f}, {ty:3.5f})" + 
                        # f"\ncmd_v: {v:3.5f}, cmd_omega: {omega:3.5f}" + 
                        "\n")

        # Data saving
        if self.save_data == 1:
            self.current_time = rospy.Time.now().to_sec()
            self.time_list.append(self.current_time)
            self.x_list.append(x)
            self.y_list.append(y)
            self.yaw_list.append(yaw)
            self.target_x_list.append(tx)
            self.target_y_list.append(ty)
            # self.target_yaw_list.append(self.target_yaw)

    def pure_pursuit_controller(self):
        target_speed = 1 # [m/s]

        target_course = TargetCourse(self.cx, self.cy)
        self.target_ind, _ = target_course.search_target_index(self.state)

        # Update data saving
        self.target_x, self.target_y = self.cx[self.target_ind], self.cy[self.target_ind]

        # Calc control input
        ai = proportional_control(target_speed, self.state.v)
        delta, self.target_ind = pure_pursuit_steer_control(self.state, target_course, self.target_ind)

        self.state.input(ai, delta, self.cmd_vel_publisher)  # Control vehicle
        if len(self.cx) - 1 > self.target_ind:
            print("Yes!!!!")
            self.state.input(ai, delta, self.cmd_vel_publisher, stop=True)

        assert len(self.cx) - 1 >= self.target_ind, "Cannot goal"

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
        plt.plot(self.target_x_list, self.target_y_list, label='Target Poses')
        plt.plot(self.cx, self.cy, label='Planned Path')
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


"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""

class State:
    '''
    This class defines the state of the vehicle
    '''
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def input(self, a, delta, publisher:rospy.Publisher, stop=False):
        # Publish the new command v and omega
        cmd_omega = self.v / WB * np.tan(delta) * 2
        cmd_v = self.v + a * dt
        print(f"cmd_v: {cmd_v}, cmd_omega: {cmd_omega}")
        if publisher is not None:
            if stop:
                publisher.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
            publisher.publish(Twist(Vector3(cmd_v, 0, 0), Vector3(0, 0, cmd_omega)))

    def update(self, x, y, yaw, v, omega):
        # Update the current x, y, psi, v and omega
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.omega = omega

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return np.hypot(dx, dy)

def proportional_control(target, current):
    a = Kp * (target - current)

    return a

def simple_controller(x, y, tx, ty, yaw=0.0, tyaw=0.0):
    # https://www.bilibili.com/video/BV19C4y1U7TE/?share_source=copy_web&vd_source=53bbd60e60dc232b7e76c75b2d1024c5
    # Compute pose error
    ex     = tx - x
    ey     = ty - y
    etheta = TrajectoryTracker.wrap_angle(tyaw - yaw)

    # Map from global to robot frame in polar coordinates
    # The linear distance to the goal
    rho = np.sqrt(ex**2 + ey**2)  
    # The angle between the goal theta and the current position of the robot
    beta = np.arctan2(ey, ex)
    # Intended angle between the robot and the direction of rho
    alpha = TrajectoryTracker.wrap_angle(beta - yaw)

    # Controller parameters
    # Gains
    P_rho   = 0.5# if rho > 0.1 else 0.01
    P_beta  = 3# if rho < 0.1 else 0
    # adjusting alpha is useless if too close to the goal
    P_alpha = 2# if rho > 0.1 else 0
    print(f"rho: {rho}, alpha: {alpha/np.pi}pi, beta: {beta/np.pi}pi")

    # Orientation considerations
    # NOTE that to take this into account, we need the measured angle between -pi and pi, i.e. the standard odometry return
    if alpha > -np.pi / 2 and alpha < np.pi / 2:
        print("toward")
        # We define that the robot is facing the goal
        v = P_rho * rho
        omega = P_alpha * alpha + P_beta * beta
    else:
        # The robot is facing the opposite direction
        print("backward")
        v = -P_rho * rho
        sign_alpha = 1 if alpha < 0 else -1
        omega = P_alpha * sign_alpha * (np.pi - abs(alpha)) + P_beta * beta

    v = np.clip(v, -1, 1)
    omega = np.clip(omega, -0.5, 0.5)
    
    print(f"v: {v}, omega: {omega}")
    return v, omega

class TargetCourse:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state:State):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state:State, trajectory:TargetCourse, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = np.arctan2(ty - state.y, tx - state.x) - state.yaw
    delta = np.arctan2(2.0 * WB * np.sin(alpha) / Lf, 1.0)

    # v, omega = simple_controller(state.x, state.y, tx, ty)

    return delta, ind


if __name__ == '__main__':
    controller = TrajectoryTracker()
    rospy.spin()
