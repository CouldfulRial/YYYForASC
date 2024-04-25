#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math
import tf

class PurePursuitController:
    def __init__(self):
        rospy.init_node('pure_pursuit_controller')

        # Subscribers
        self.path_subscriber = rospy.Subscriber('/path', Path, self.path_callback)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Controller parameters
        self.look_ahead_distance = 0.5  # meters
        self.current_path = None
        self.current_pose = None

    def path_callback(self, msg):
        """Callback function for path updates."""
        self.current_path = msg

    def odom_callback(self, msg):
        """Callback function for odometry updates."""
        self.current_pose = msg.pose.pose

        # Execute the control loop
        self.control_loop()

    def control_loop(self):
        """Main control loop for calculating and publishing velocity commands."""
        if not self.current_path or not self.current_pose:
            return  # Do nothing if no path or pose yet

        # Find the closest path point ahead of the robot by at least the look-ahead distance
        closest_point = None
        min_distance = float('inf')

        for pose in self.current_path.poses:
            dx = pose.pose.position.x - self.current_pose.position.x
            dy = pose.pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)

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

        # Publish the velocity command
        self.cmd_vel_publisher.publish(velocity_command)
        rospy.loginfo("Published velocity command: {}".format(velocity_command))

if __name__ == '__main__':
    try:
        controller = PurePursuitController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
