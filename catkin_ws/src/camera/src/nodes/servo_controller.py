#! /usr/bin/env python
'''
This node controls the servos for the camera to the desired angles.
Channel 3 for pan camera.
    [-90, 90]deg --> [850, 2200]us
Channel 4 for tilt
    [0, 90]deg --> [500, 1500]us

Subscribed topics:
set_servo_angle        [asclinic_pkg/LeftRightFloat32] (left=pan, right=tilt)
Published topics:
/asc/set_servo_pulse_width  [asclinic_pkg/ServoPulseWidth]
'''

import rospy
import numpy as np
from asclinic_pkg.msg import LeftRightFloat32, ServoPulseWidth

class ServoController:
    def __init__(self):
        # Initialise node
        self.node_name = 'servo_controller'
        rospy.init_node(self.node_name, anonymous=True)

        # Initialise the subscribers
        self.sub_angle = rospy.Subscriber('set_servo_angle', LeftRightFloat32, self.update_angle_callback)

        # Timer: Calls the timer_callback function at 1 Hz
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        # Initialise the publisher
        self.pulse_width_pub = rospy.Publisher("/asc/set_servo_pulse_width", ServoPulseWidth, queue_size=10)

        # Reset the servos
        rate = rospy.Rate(10)  # Define the rate of publishing (10 Hz in this case)
        start_time = rospy.Time.now()  # Record the start time
        duration = rospy.Duration(2)  # Set the duration for which to publish
        
        while rospy.Time.now() - start_time < duration:
            self.publish_pan(0)
            self.publish_tilt(0)
            rate.sleep()  # Sleep to maintain the publishing rate

        
    def update_angle_callback(self, data):
        # Update the angles
        pan_angle = data.left
        tilt_angle = data.right

        # Publish
        self.publish(pan_angle, tilt_angle)

    def timer_callback(self, event):
        self.publish_pan(0)
        self.publish_tilt(0)

    def publish(self, pan_ang, tilt_ang):
        # Convert the angles to pulse width
        pan_pulse_width = ServoController.pan_get_pulse_width(pan_ang)
        tilt_pulse_width = ServoController.tilt_get_pulse_width(tilt_ang)

        # Publish the pulse width
        # Pan servo
        self.pulse_width_pub.publish(
            ServoPulseWidth(
                channel=3,
                pulse_width_in_microseconds=pan_pulse_width)
        )
        # Tilt servo
        self.pulse_width_pub.publish(
            ServoPulseWidth(
                channel=4,
                pulse_width_in_microseconds=tilt_pulse_width)
        )

    def publish_pan(self, pan_ang):
        # Convert the angles to pulse width
        pan_pulse_width = ServoController.pan_get_pulse_width(pan_ang)

        # Publish the pulse width
        # Pan servo
        self.pulse_width_pub.publish(
            ServoPulseWidth(
                channel=3,
                pulse_width_in_microseconds=pan_pulse_width)
        )

    def publish_tilt(self, tilt_ang):
        # Convert the angles to pulse width
        tilt_pulse_width = ServoController.tilt_get_pulse_width(tilt_ang)

        # Publish the pulse width
        # Tilt servo
        self.pulse_width_pub.publish(
            ServoPulseWidth(
                channel=4,
                pulse_width_in_microseconds=tilt_pulse_width)
        )

    @staticmethod
    def pan_get_pulse_width(angle):
        # [-90, 90]deg --> [850, 2200]us
        angle = np.clip(angle, -90, 90)
        pw = angle * 7.5 + 1525
        return int(pw)
            
    @staticmethod
    def tilt_get_pulse_width(angle):
        # [0, 90]deg --> [500, 1500]us
        angle = np.clip(angle, 0, 90)
        pw = angle * 10 + 500
        return int(pw)
        
if __name__ == '__main__':
    servo_controller = ServoController()
    rospy.spin()
