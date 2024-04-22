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
ERROR_LINEAR = 0.1
ERROR_ANGULAR = pi / 100

class TaskLevelController:
    def __init__(self):
        rospy.init_node('task_level_controller', anonymous=True)

        # Subscribed topics
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Timer: Calls the timer_callback function at 10 Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # Published topics:
        self.pos_pub = rospy.Publisher('ref_pose', Pose2D, queue_size=10)

        # Initialise parameters
        self.current_pose_x = 0
        self.current_pose_y = 0
        self.current_theta  = 0
        self.target = Pose2D(0, 0, 0)

        # Initialise states
        # self.prev_state = "MOVE_FORWARD"
        self.state = "MOVE_FORWARD"

    def timer_callback(self, event):
        pass

    def odom_callback(self, data):
        # Extract the current pose from the odometry message
        self.current_pose_x = data.pose.pose.position.x
        self.current_pose_y = data.pose.pose.position.y
        self.current_theta = data.pose.pose.orientation.z
        # self.current_theta = self.quat_to_euler(current_orientation)
        # map to [0, 2pi]
        # if self.current_theta < 0:
        #     self.current_theta += 2 * pi

        # State transition logic
        if self.state == "MOVE_FORWARD" and self.reached_target(3, 0, 0):
            self.state = "TURN_AROUND"

        elif self.state == "TURN_AROUND" and self.reached_target(3, 0, pi):
            self.state = "MOVE_BACKWARD"

        elif self.state == "MOVE_BACKWARD" and self.reached_target(0, 0, pi):
                self.state = "STOP"

        else:
            self.state = self.state

        # State action logic
        if self.state == "MOVE_FORWARD":
            self.target = Pose2D(3, 0, 0)

        elif self.state == "TURN_AROUND":
            self.target = Pose2D(3, 0, pi)

        elif self.state == "MOVE_BACKWARD":
            self.target = Pose2D(0, 0, pi)

        else:  # Stop
            self.target = Pose2D(0, 0, pi)

        # Publish the target pose
        self.pos_pub.publish(self.target)
        rospy.loginfo("-"*25 + "FSM" + "-"*25 + 
                      f"\nState: {self.state}" + 
                      f"\ntarger pose: {self.target.x:3.2f}, {self.target.y:3.2f}, {self.target.theta:3.2f},")

    @staticmethod
    def quat_to_euler(quat):
        if quat is not None:
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            return euler[2]  # return the yaw, which is the orientation around z-axis
        else:
            return 0  # If intialised to None, return 0

    def reached_target(self, x, y, theta):
        if (abs(self.current_pose_x - x) < ERROR_LINEAR and
            abs(self.current_pose_y - y) < ERROR_LINEAR and
            abs(self.current_theta - theta) < ERROR_ANGULAR):
            return True
        return False
      

if __name__ == '__main__':
    try:
        controller = TaskLevelController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
