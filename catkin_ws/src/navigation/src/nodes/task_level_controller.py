#!/usr/bin/env python
'''
This node is for state-level control
Subscribed topics:
    odom                      [nav_msgs/Odometry]
Published topics:
    ref_pose                  [geometry_msgs/Pose2D]
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import tf.transformations
from math import pi

# Define hyperparameters
ERROR_X = 0.1
ERROR_Y = 0.4
ERROR_ANGULAR = pi / 50

# Path
PATH = [
    Pose2D(0, 0, 0)#, Pose2D(9, 0, pi), Pose2D(0, 0, pi)
]

class TaskLevelController:
    def __init__(self):
        rospy.init_node('task_level_controller', anonymous=True)

        # Subscribed topics
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Timer: Calls the timer_callback function at 2 Hz
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        # Published topics:
        self.pos_pub = rospy.Publisher('ref_pose', Pose2D, queue_size=10)

        # Initialise parameters
        self.current_pose_x = 0
        self.current_pose_y = 0
        self.current_theta  = 0
        self.target = Pose2D(0, 0, 0)

        # Initialise states
        self.state = 0

    def timer_callback(self, event):
        # State action
        self.target = PATH[self.state]
        if self.reached_target(PATH[self.state]):
            # State transition
            self.state += 1
            if self.state == len(PATH):
                self.state -= 1

        self.pos_pub.publish(self.target)
        # rospy.loginfo("-"*25 + "FSM" + "-"*25 + 
        #               f"\nState: {self.state}" + 
        #               f"\ntarger pose: {self.target.x:3.2f}, {self.target.y:3.2f}, {self.target.theta:3.2f},")

    def odom_callback(self, data):
        # Extract the current pose from the odometry message
        self.current_pose_x = data.pose.pose.position.x
        self.current_pose_y = data.pose.pose.position.y
        self.current_theta = data.pose.pose.orientation
        self.current_theta = self.quat_to_euler(self.current_theta)

    @staticmethod
    def quat_to_euler(quat):
        if quat is not None:
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            return euler[2]  # return the yaw, which is the orientation around z-axis
        else:
            return 0  # If intialised to None, return 0

    def reached_target(self, pose:Pose2D):
        if (abs(self.current_pose_x - pose.x) < ERROR_X and
            abs(self.current_pose_y - pose.y) < ERROR_Y and
            abs(self.wrap_angle(pose.theta - self.current_theta)) < ERROR_ANGULAR):
            return True
        return False
    
    @staticmethod
    def wrap_angle(angle):
        # Angle wrapping
        return (angle + pi) % (2 * pi) - pi

if __name__ == '__main__':
    try:
        controller = TaskLevelController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
