#!/usr/bin/env python

import rospy
from navigation.msg import LRFloat32, LRInt32
import csv
import os

def store_data_to_csv(sequence, time, left_wheel_reading, right_wheel_reading, filename):
    # Check if the file exists to determine if we need to write headers
    file_exists = os.path.isfile(filename)

    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)

        # If the file does not exist, write the header
        if not file_exists:
            writer.writerow(['Sequence', 'time', 'Left Wheel Reading', 'Right Wheel Reading'])

        # Write the data
        writer.writerow([sequence, time, left_wheel_reading, right_wheel_reading])


# Define parameters
DUTY_CYCLE = 20
COUNT_PER_REVOLUTION = 1120
ANGLE_PER_COUNT = 360 / COUNT_PER_REVOLUTION
DELTA_T = 0.1  # The interval which the encoder counts are published


class RotateByTime:
    def __init__(self):
        rospy.init_node('rotate_by_time', anonymous=True)

        # Get user parameter
        self.wheel = rospy.get_param('~wheel', 'default_value')
        self.target_time = rospy.get_param('~target_time', 'default_value')
        self.file = rospy.get_param('~target_file', 'default_value')
        self.target_file = "".join(["/home/asc/YYYForASC/catkin_ws/src/navigation/src/data/",self.file])
        rospy.loginfo(f"wheel: {self.wheel}, target time: {self.target_time}, target file location: {self.target_file}")

        # Subscribers
        # This subscriber will listen to the encoder_counts topic, in order to count the accumulated encoder counts
        self.sub_ec = rospy.Subscriber('encoder_counts', LRInt32, self.encoder_callback)

        # Publisher
        # This publisher will publish the duty cycle to the i2c_for_motor node
        self.pub_dc = rospy.Publisher('set_motor_duty_cycle', LRFloat32, queue_size=10)
        
        # Initialize sequence size, increment
        self.seq_size = 0


    def encoder_callback(self, data):
        # initialise publishing
        set_dc = LRFloat32()
        set_dc.left = DUTY_CYCLE if self.wheel == 'left' else 0
        set_dc.right = DUTY_CYCLE if self.wheel == 'right' else 0

        # Publish the duty cycle
        self.pub_dc.publish(set_dc)

        # Listen to encoder
        encoder_left = data.left
        encoder_right = data.right  # By our convention, left and right is w.r.t. the robot's perspective
        encoder_seq = data.seq_num
        if encoder_left == 0 and encoder_right == 0:
            # Only start counting when the encoder starts to move
            self.seq_size = 0
        time = self.seq_size * DELTA_T
        
        # Publish and save the data
        rospy.loginfo(f"{encoder_seq}, {time:.2f} seconds, Left: {encoder_right:3d}, Right: {encoder_left:3d}")
        store_data_to_csv(encoder_seq, time, encoder_left, encoder_right, self.target_file)

        # Time has reached, stop the motor and close the node
        if self.seq_size >= self.target_time / DELTA_T:
            set_dc.left = 0
            set_dc.right = 0
            self.pub_dc.publish(set_dc)
            rospy.signal_shutdown('Time has reached. Node shutting down...')
        else:
            self.seq_size += 1
        

if __name__ == '__main__':
    try:
        RBA = RotateByTime()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
