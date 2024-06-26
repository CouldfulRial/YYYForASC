#!/usr/bin/env python

'''
This node implements the pure pursuit controller
Subscribed topics:
    path    [nav_msgs/Path]
    odom    [nav_msgs/Odometry]
Published topics:
    cmd_vel [geometry_msgs/Twist]
    tar_pt  [geometry_msgs/PointStamped]  # The current target point for visualisation
    tar_reached [std_msgs/Bool]  # The target has been reached
'''
import rospy
import rospkg
import datetime
import csv
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Vector3, Point, PointStamped, PoseStamped, Pose
from std_msgs.msg import Bool, String
import tf
import matplotlib.pyplot as plt
import numpy as np
import numpy as np

# Parmeters
TIME_STEP = 0.1  # s
k = 0#25  # look forward gain
Lfc = 1  # [m] look-ahead distance
points  = [(7.5, 4), (0, 4), (-1, 3), (-1, -1.75), (1, -3.75), (6, -3.75), (7, -2.75), (7, 2), (5, 2), (5, -1), (4, -2), (3, -1), (3, 2), (1, 2), (1, -2)]
centres = [None,     (0, 3), None,    (1, -1.75),  None,       (6, -2.75), None,       (6, 2), None,   (4, -1), (4, -1), None,    (2, 2), None           ]
dx = 0.7

## Controller
# P
P_rho = 2
P_beta = 0
P_alpha = 3
# I
I_rho = 0
I_beta = 0#0.00001
I_alpha = 0.00001
# D
D_rho = 0
D_beta = 0#10
D_alpha = 10

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

    def input(self, a, delta):
        # Publish the new command v and omega
        cmd_omega = delta
        cmd_v = self.v + a * TIME_STEP

        return cmd_v, cmd_omega

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
                try:
                    distance_next_index = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                except IndexError:
                    break
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx)-1 else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

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
        self.Lf = 0.0
        self.state = State(x=7.5, y=4, yaw=np.pi, v=0.0)
        self.target_x = self.target_y = 0.0
        self.target_ind = 0
        self.v = self.omega = 0.0
        self.ini_time = rospy.Time.now()
        self.current_time = 0.0
        self.fsm_state = "STOP"
        if self.simulate == 1:
            self.beta_sig = 0.0

        # Initialise the trajectory
        self.trajectory = traj(points, dx, centres)
        self.cx = [p[0] for p in self.trajectory]
        self.cy = [p[1] for p in self.trajectory]
        self.target_course = TargetCourse(self.cx, self.cy)

        # Controller variables
        self.rho       = self.beta       = self.alpha       = 0.0
        self.rhoi      = self.betai      = self.alphai      = 0.0
        self.rhod      = self.betad      = self.alphad      = 0.0
        self.last_rho  = self.last_beta  = self.last_alpha  = 0.0

        # Publisher
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.tar_pt_publisher = rospy.Publisher('tar_pt', PointStamped, queue_size=10)
        self.tar_reached_publisher = rospy.Publisher('tar_reached', Bool, queue_size=10)
        self.path_pub = rospy.Publisher('path', Path, queue_size=10)

        # Subscribers
        # self.path_subscriber = rospy.Subscriber('path', Path, self.path_callback)
        self.odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.state_sub = rospy.Subscriber('state', String, self.state_callback)

        # Timer: Calls the timer_callback function at 10 Hz
        self.timer = rospy.Timer(rospy.Duration(TIME_STEP), self.timer_callback)
        self.debugger = rospy.Timer(rospy.Duration(2), self.debugger_callback)

        # Register the shutdown callback
        rospy.on_shutdown(self.shutdown_callback)

        # Intialise data saving
        if self.save_data == 1:
            self.time_list = []
            self.x_list = []
            self.y_list = []
            self.yaw_list = []
            self.target_x_list = []
            self.target_y_list = []
            self.target_yaw_list = []
            self.omega_list = []
            # Get save path. Set the file name to the intial time
            self.save_path = self.get_save_path()
            initial_time = datetime.datetime.now()
            self.formatted_time = initial_time.strftime('%y%m%d_%H_%M_%S')

    def debugger_callback(self, event):
        if self.simulate == 1:
            self.beta_sig = np.pi / 2# if self.beta_sig == 0.0 else 0.0

    def path_callback(self, data:Path):
        return
        # Extract the planned path to self.cx, self.cy
        self.cx = np.array([point.pose.position.x for point in data.poses])
        self.cy = np.array([point.pose.position.y for point in data.poses])
        self.target_course = TargetCourse(self.cx, self.cy)
        self.target_ind = 0

    def odom_callback(self, data:Odometry):
        # Update the robot state
        self.state.update(x=data.pose.pose.position.x, 
                          y=data.pose.pose.position.y, 
                          yaw=self.quat_to_euler(data.pose.pose.orientation), 
                          v=data.twist.twist.linear.x, 
                          omega=data.twist.twist.angular.z)

    def state_callback(self, data:String):
        self.fsm_state = data.data

    def timer_callback(self, event):
        # Execute the control loop
        self.v, self.omega = self.pure_pursuit_controller()
        if self.fsm_state != "SEARCH":
            self.v = self.omega = 0.0

        # Logging
        if self.verbosity == 1:
            rospy.loginfo("-"*20 + self.node_name + "-"*20 +
                        f"\n\nRobot State: {self.current_time:.2f}s, debug: {self.simulate}"
                        f"\nx: {self.state.x:3.5f}, y: {self.state.y:3.5f}, yaw: {self.state.yaw/np.pi:3.5f}pi" +
                        f"\nv: {self.state.v:3.5f}" +
                        "\n\nCurrent Targeting: "
                        f"\nind: {self.target_ind}, Lf: {self.Lf:3.5f}" +
                        f"\ntarget: ({self.target_x:3.5f}, {self.target_y:3.5f}, {self.beta/np.pi:3.5f}pi)" + 
                        "\n\nController Variables: "
                        "\nrho: "
                        f"\nP:{self.rho:3.5f}, I:{self.rhoi:3.5f}, D:{self.rhod:3.5f}" + 
                        "\nbeta: "
                        f"\nP:{self.beta:3.5f}, I:{self.betai:3.5f}, D:{self.betad:3.5f}" + 
                        "\nalpha: "
                        f"\nP:{self.alpha:3.5f}, I:{self.alphai:3.5f}, D:{self.alphad:3.5f}" + 
                        f"\ncmd_v: {self.v:3.5f}, cmd_omega: {self.omega:3.5f}" + 
                        "\n")

        # Publish the control
        self.cmd_vel_publisher.publish(
            Twist(
                Vector3(self.v, 0, 0),
                Vector3(0, 0, self.omega)
            )
        )
        
        # Publish the path
        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = 'map'
        self.path_msg.poses = [PoseStamped(pose=Pose(position=Point(x=x, y=y))) for x, y in self.trajectory]
        self.path_pub.publish(self.path_msg)

        # Publish the target point
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.point.x = self.target_x
        msg.point.y = self.target_y
        msg.point.z = 0.0
        self.tar_pt_publisher.publish(msg)

        # Data saving
        if self.save_data == 1 and self.state.x != 0.0:
            self.current_time = (rospy.Time.now() - self.ini_time).to_sec()
            self.time_list.append(self.current_time)
            self.x_list.append(self.state.x)
            self.y_list.append(self.state.y)
            self.yaw_list.append(self.state.yaw)
            self.target_x_list.append(self.target_x)
            self.target_y_list.append(self.target_y)
            self.target_yaw_list.append(self.beta)
            self.omega_list.append(self.omega)

    def pure_pursuit_controller(self):
        if (self.target_ind >= len(self.cx)-1) and (self.rho < 0.5):
            # Announce that the target is reached
            self.tar_reached_publisher.publish(Bool(True))
            rospy.sleep(1)
            return 0.0, 0.0
        self.tar_reached_publisher.publish(Bool(False))

        self.target_ind, _ = self.target_course.search_target_index(self.state)

        # Update data saving
        self.target_x, self.target_y = self.cx[self.target_ind], self.cy[self.target_ind]

        # Calc control input
        v, omega, self.target_ind = self.pure_pursuit_steer_control(self.state, self.target_course, self.target_ind)

        v = np.clip(v, -0.2, 0.2)
        omega = np.clip(omega, -np.pi, np.pi)
        return v, omega

    def pure_pursuit_steer_control(self, state:State, trajectory:TargetCourse, pind):
        ind, self.Lf = trajectory.search_target_index(state)

        if pind >= ind:
            ind = pind

        ## Get the current target point
        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1

        ## Get the current vehicle position
        x = state.x
        y = state.y

        ## Get error position
        ex = tx - x
        ey = ty - y

        ## P
        # The linear distance to the goal
        self.rho = np.hypot(ex, ey)
        # The angle between the goal theta and the current position of the robot
        self.beta = np.arctan2(ey, ex)
        if self.simulate == 1:
            self.beta = self.beta_sig
        # Intended angle between the robot and the direction of rho
        self.alpha = TrajectoryTracker.wrap_angle(self.beta - state.yaw)
    
        ## I
        self.rhoi += self.rho
        self.betai += self.beta
        self.alphai += self.alpha

        ## D
        self.rhod = self.rho - self.last_rho
        self.betad = self.beta - self.last_beta
        self.alphad = self.alpha - self.last_alpha

        # Orientation consideration
        # if self.alpha > -np.pi / 2 and self.alpha < np.pi / 2:
        #     pass
        # else:
        #     self.rho = -self.rho
        #     sign_alpha = 1 if self.alpha < 0 else -1
        #     self.alpha = sign_alpha * (np.pi - abs(self.alpha))

        # Limit speed if trying to steering. Hence, the gain should inversly p to abs(alpha)
        # P = P_rho * (1 - abs(self.alpha) / np.pi)
        # P = 1 / (abs(self.alpha) + 1 / P_rho)

        if abs(self.alpha) > 0.17:
            P = 0.1
        else:
            P = 0.4

        ## Calculate the inputs to the robot
        acc   = P   * self.rho   + I_rho   * self.rhoi   + D_rho   * self.rhod
        delta = P_beta  * self.beta  + I_beta  * self.betai  + D_beta  * self.betad + \
                P_alpha * self.alpha + I_alpha * self.alphai + D_alpha * self.alphad

        ## Return the control inputs
        # v, omega = state.input(acc, delta)
        
        v = acc
        omega = delta #2 * np.sin(self.alpha) / self.Lf * 0.25 + 0.1 * self.alphad

        ## Update the error terms
        self.last_rho = self.rho
        self.last_beta = self.beta
        self.last_alpha = self.alpha

        return v, omega, ind

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

    def shutdown_callback(self):
        self.verbosity = 0
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
        plt.plot(self.x_list[10:], self.y_list[10:], label='Robot Path')
        plt.plot(self.target_x_list, self.target_y_list, label='Target Poses')
        plt.plot(self.cx, self.cy, label='Planned Path')
        # Plot the directions
        TrajectoryTracker.plot_vectors(self.x_list[10:], self.y_list[10:], self.yaw_list[10:])

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
        plt.plot(self.time_list, self.target_yaw_list, label='Yaw Target')
        # plt.plot(self.time_list, self.omega_list, label='Omega')
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
            writer.writerow(['Time (s)', 
                             'X Position (m)', 
                             'Y Position (m)', 
                             "Yaw (rad)", 
                             "Omega (rad/s)"])
            for i in range(len(self.time_list)):
                writer.writerow([self.time_list[i], 
                                 self.x_list[i], 
                                 self.y_list[i], 
                                 self.yaw_list[i], 
                                 self.omega_list[i]])

    @staticmethod
    def get_save_path():
        rospack = rospkg.RosPack()
        return rospack.get_path("controller") + '/src/data/'

def traj(points, dx, centres):
    # This function is used to generate a trajectory from a list of points, the returned list is a list of points that points are inserted in between the points of the input list with a distance of dx between them.
    trajectory = []
    for i in range(len(points)-1):
        curr = points[i]
        next = points[i+1]

        generated_points = generate_pts_between(curr, next, dx, centres[i])
        print(f"Generated points between {curr} and {next}: \n{generated_points}")
        trajectory.extend(generated_points)

    return trajectory

def generate_pts_between(curr, next, dx, centre=None):
    if centre is None:
        return generate_linear_points(curr, next, dx)
    
    # Calculate the radius of the circle
    return generate_arc_points(curr, next, centre, dx)
        
def generate_linear_points(a, b, d):
    # Calculate the total distance between points a and b
    total_distance = np.linalg.norm(np.array(b) - np.array(a))
    
    # Calculate the number of points needed
    num_points = int(total_distance / d) + 1

    # Generate the points
    points = []
    for i in range(num_points + 1):
        t = i / num_points
        x = a[0] + t * (b[0] - a[0])
        y = a[1] + t * (b[1] - a[1])
        points.append((x, y))
    
    return points

def generate_arc_points(a, b, centre, d):
    def distance(p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def angle_between_points(p, centre):
        return np.arctan2(p[1] - centre[1], p[0] - centre[0])

    # Calculate the radius
    radius = distance(a, centre)

    # Calculate start and end angles
    start_angle = angle_between_points(a, centre)
    end_angle = angle_between_points(b, centre)

    # Ensure the angles are in the correct range for the shortest arc
    if end_angle < start_angle:
        end_angle += 2 * np.pi

    # Calculate the angle difference
    angle_diff = end_angle - start_angle

    # Adjust if the angle difference exceeds π to get the shortest arc
    if angle_diff > np.pi:
        start_angle, end_angle = end_angle, start_angle + 2 * np.pi

    # Calculate the arc length
    arc_length = radius * (end_angle - start_angle)

    # Calculate the number of points needed
    num_points = int(arc_length / d) + 1

    # Calculate the angle increment
    angle_increment = (end_angle - start_angle) / (num_points - 1)

    # Generate points on the arc
    points = []
    for i in range(num_points):
        angle = start_angle + i * angle_increment
        x = centre[0] + radius * np.cos(angle)
        y = centre[1] + radius * np.sin(angle)
        points.append((x, y))

    if a == (5, -1) or a == (4, -2):
        points.reverse()

    return points

if __name__ == '__main__':
    controller = TrajectoryTracker()
    rospy.spin()
