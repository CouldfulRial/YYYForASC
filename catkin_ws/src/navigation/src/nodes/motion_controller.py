#!/usr/bin/env python

'''
This node controls the motion (outer controller)
Subscribed topics:
    /asc/odom                 [nav_msgs/Odometry]
    ref_pose                  [geometry_msgs/Pose2D]
Published topics:
    reference_wheel_speeds    [asclinic_pkg/asclinc_pkg]
'''

import rospy
from math import pi, sqrt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from asclinic_pkg.msg  import LeftRightFloat32

class MotionController:
    def __init__(self):
        rospy.init_node('motion_controller', anonymous=True)

        # Subscriber to the Odometry messages
        self.odom_sub = rospy.Subscriber('/asc/odom', Odometry, self.odom_callback)

        # Publisher for the reference wheel speeds
        self.speed_pub = rospy.Publisher('reference_wheel_speeds', LeftRightFloat32, queue_size=10)

        # Desired pose, later will be topic
        self.desired_pose = Pose2D(x=0, y=5.0, theta=0)

        # Controller gains
        self.Kp_pos = 0.5  # Position gain
        self.Kp_ang = 0.3  # Angular gain

    def odom_callback(self, data):
        # Extract the current pose from the odometry message
        current_pose_x = data.pose.pose.position.x
        current_pose_y = data.pose.pose.position.y
        current_orientation = data.pose.pose.orientation

        # Convert quaternion to Euler angles (assuming a function quat_to_euler exists)
        current_theta = self.quat_to_euler(current_orientation)

        # Compute pose error
        error_x = self.desired_pose.x - current_pose_x
        error_y = self.desired_pose.y - current_pose_y
        error_theta = self.desired_pose.theta - current_theta

        # Compute control signals
        # Simple proportional controller for demonstration
        v = self.Kp_pos * sqrt(error_x**2 + error_y**2)
        omega = self.Kp_ang * error_theta

        # Assuming a differential drive robot
        # Convert from linear and angular velocity to wheel speeds
        wheel_radius = 0.1  # Example wheel radius
        wheel_base = 0.5  # Example distance between wheels
        left_speed = (2*v - omega*wheel_base) / (2*wheel_radius)
        right_speed = (2*v + omega*wheel_base) / (2*wheel_radius)

        # Publish the calculated wheel speeds
        self.speed_pub.publish(LeftRightFloat32(left=left_speed, right=right_speed))

    def quat_to_euler(self, quat):
        # Import necessary function
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler[2]  # return the yaw, which is the orientation around z-axis

if __name__ == '__main__':
    try:
        controller = MotionController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
