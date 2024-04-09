#!/usr/bin/env python

import rospy
from navigation.msg import LRFloat32, LRInt32

# Define parameters
DUTY_CYCLE = 25
COUNT_PER_REVOLUTION = 1120
ANGLE_PER_COUNT = 360 / COUNT_PER_REVOLUTION
TARGET_ANGLE = 15


class RotateByAngle:
    def __init__(self):
        rospy.init_node('encoder_counter', anonymous=True)
        self.subscriber = rospy.Subscriber('encoder_counts', LRInt32, self.callback)
        
        # Initialize accumulated counts
        self.accumulated_left = 0
        self.accumulated_right = 0

    def callback(self, data):
        # Update accumulated counts
        self.accumulated_left += data.left
        self.accumulated_right += data.right

        # Display the accumulated counts and sequence number in the console
        rospy.loginfo(f"Seq_Num: {data.seq_num}, Accumulated Left: {self.accumulated_left}, Accumulated Right: {self.accumulated_right}")


# class RotateByAngle:
#     def __init__(self):
#         rospy.init_node('rotate_by_angle', anonymous=True)

#         # Subscribers
#         # This subscriber will listen to the encoder_counts topic, in order to count the accumulated encoder counts
#         self.sub_ec = rospy.Subscriber('encoder_counts', LRInt32, self.encoder_callback)
#         # This subscriber listens to manual input of target angles
#         self.sub_ta = rospy.Subscriber('target_angles', LRInt32, self.angle_callback)

#         # Publisher
#         # This publisher will publish the duty cycle to the i2c_for_motor node
#         self.pub_dc = rospy.Publisher('set_motor_duty_cycle', LRFloat32, queue_size=10)
        
#         # Initialize accumulated angles
#         self.left_angle = 0
#         self.right_angle = 0

#         # Initialize target reached
#         self.target_reached = False


#     def encoder_callback(self, data):
#         if self.target_reached:
#             return

#         # Update accumulated counts
#         self.left_angle += data.right * ANGLE_PER_COUNT
#         self.right_angle += data.left * ANGLE_PER_COUNT

#         # Display the accumulated counts and sequence number in the console
#         rospy.loginfo(f"Seq_Num: {data.seq_num}, Left: {self.left_angle}, Right: {self.right_angle}")

#         # Determine if the target angle has been reached
#         left_reached = self.left_angle >= TARGET_ANGLE
#         right_reached = self.right_angle >= TARGET_ANGLE

#         # Prepare the published msg
#         set_dc = LRFloat32()

#         set_dc.left = DUTY_CYCLE
#         if left_reached:
#             rospy.loginfo("Left target angle reached. Stopping left motor.")
#             set_dc.left = 0

#         set_dc.right = DUTY_CYCLE
#         if right_reached:
#             rospy.loginfo("Right target angle reached. Stopping right motor.")
#             set_dc.right = 0
        
#         # set_dc.left = 0 if left_reached else DUTY_CYCLE
#         # set_dc.right = 0 if right_reached else DUTY_CYCLE

#         # Publish the duty cycle
#         self.pub_dc.publish(set_dc)

#         # Update target reached
#         if left_reached and right_reached:
#             rospy.loginfo("Target angles reached. Stopping motors and shutting down node.")
#             self.target_reached = True
#             # Gracefully shutdown the node
#             rospy.signal_shutdown("Target angles reached")

#     def angle_callback(self, data):
#         self.target_left = data.left
#         self.target_right = data.right
        

if __name__ == '__main__':
    try:
        RBA = RotateByAngle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
