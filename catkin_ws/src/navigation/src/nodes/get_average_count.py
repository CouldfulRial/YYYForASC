#!/usr/bin/env python

import rospy
from navigation.msg import LRFloat32, LRInt32

# Define parameters
DELTA_T = 0.1
ENTRIES = 10 / DELTA_T


class AvgCounts:
    def __init__(self):
        rospy.init_node('get_average_count', anonymous=True)
        self.subscriber = rospy.Subscriber('encoder_counts', LRInt32, self.callback)
        
        # Initialize averaged counts
        self.avg_left = 0
        self.avg_right = 0

        # Initialise the number of entries
        self.num_entry = 0

    def callback(self, data):
        # Update the number of entries
        self.num_entry += 1

        # Update the average
        self.avg_left = (self.avg_left * (self.num_entry - 1) + data.left) / self.num_entry
        self.avg_right = (self.avg_right * (self.num_entry - 1) + data.right) / self.num_entry

        # If the number of entries is ENTRIES, shutdown the node
        if self.num_entry == ENTRIES:
            rospy.loginfo(f"averaged Left: {self.avg_left}, averaged Right: {self.avg_right}")
            rospy.signal_shutdown("Number of entries reached")

        # Display in the console
        rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, averaged Left: {self.avg_left:.2f}, averaged Right: {self.avg_right:.2f}")
        

if __name__ == '__main__':
    try:
        AC = AvgCounts()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
