#! /usr/bin/env python
'''
This node get the speeds of the wheels from the encoder counts at increasing duty cycles
Publisher
    /asc/set_motor_duty_cycle [asclinic_pkg/LeftRightFloat32]
Subscriber
    /asc/encoder_counts       [asclinic_pkg/LeftRightInt32]
'''
from math import pi
import rospy
import rospkg
import datetime

# Subscriber messages
from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32
import csv

# Constants
CPR = 1120
RADIAN_PER_COUNT = 2 * pi / CPR

TIME_STEP = 0.1  # s, which is the interval encoder publishes data


class GetCov:
    def __init__(self):
        # Initialise node
        self.node_name = 'get_wheel_cov'
        rospy.init_node(self.node_name, anonymous=True)

        # Initialise the subscribers
        self.sub_encoder = rospy.Subscriber('/asc/encoder_counts', LeftRightInt32, self.update_encoder_callback)

        # Initialise the variables
        self.delta_left_counts  = 0
        self.delta_right_counts = 0
        self.dc = 0
        self.seq = 0

        # Initialise saving
        self.save_path = self.get_save_path()
        intial_time = datetime.datetime.now()
        self.formatted_time = intial_time.strftime('%y%m%d_%H_%M_%S')

        # Register the shutdown callback
        rospy.on_shutdown(self.shutdown_callback)

        # Initialise the publisher
        self.speeds_pub = rospy.Publisher("/asc/set_motor_duty_cycle", LeftRightFloat32, queue_size=10)
        self.timer_update_dc = rospy.Timer(rospy.Duration(5), self.update_dc_callback)
        self.timer_publish_dc = rospy.Timer(rospy.Duration(0.1), self.publish_dc_callback)

    def update_encoder_callback(self, data):
        # Update the encoder counts
        self.delta_left_counts  = data.left
        self.delta_right_counts = data.right

        # log info
        rospy.loginfo("-"*25 + self.node_name + "-"*25 + 
                        f"\nseq: {self.seq}:" + 
                        f"\nduty cycle: {self.dc}" + 
                        f"\n\tleft wheel speed: {self.delta_left_counts}" +
                        f"\n\tright wheel speed: {self.delta_right_counts}")
        
        # Store the data
        with open(f'{self.save_path}get_cov_{self.formatted_time}.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([self.seq, self.dc, self.delta_left_counts, self.delta_right_counts])

        self.seq += 1
        # self.speeds_pub.publish(LeftRightFloat32(left=self.dc, right=self.dc))
    
    def publish_dc_callback(self, event):
        # Publish the duty cycle
        self.speeds_pub.publish(LeftRightFloat32(left=self.dc, right=self.dc))

    def update_dc_callback(self, event):
        # Update the duty cycles
        self.dc += 5
        # Shutdown if all duty cycles have been tested
        if self.dc > 100:
            rospy.signal_shutdown("Test Completes")
        
    def shutdown_callback(self):
        rospy.loginfo(f"Shutting down node {self.node_name}....")
        self.speeds_pub.publish(LeftRightFloat32(left=0, right=0))

    def get_save_path(self):
        rospack = rospkg.RosPack()
        return rospack.get_path("controller") + '/src/data/'

if __name__ == '__main__':
    motor_model = GetCov()
    rospy.spin()