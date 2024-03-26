#include "ros/ros.h"
#include <ros/package.h>

// Declare "member" variables
ros::Timer m_timer_for_counting;

// Declare the function prototypes
void timerCallback(const ros::TimerEvent&);

// Implement the timer callback function
void timerCallback(const ros::TimerEvent&)
{
    static uint counter = 0;
    counter++;
    // Display the current counter value to the console
    ROS_INFO_STREAM("[SPINNER CPP NODE] counter = " << counter);
}

int main(int argc, char* argv[])
{
    // Initialise the node
    ros::init(argc, argv, "spinner_cpp_node");
    // Start the node by initialising a node handle
    ros::NodeHandle nh("~");
    // Display the namespace of the node handle
    ROS_INFO_STREAM("SPINNER CPP NODE] namespace of nh = " << nh.getNamespace());
    // Initialise a timer
    float timer_delta_t_in_seconds = 0.5;
    m_timer_for_counting = nh.createTimer(ros::Duration(timer_delta_t_in_seconds), timerCallback, false);
    // Spin as a single-threaded node
    ros::spin();
    // Main has ended, return 0
    return 0;
}