// This file is part of ASClinic-System.
//
// See the root of the repository for license details.
//
// ----------------------------------------------------------------------------
//     _    ____   ____ _ _       _          ____            _                 
//    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
//   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
//  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
// /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
//                                                 |___/                       
//
// DESCRIPTION:
// C++ node as a skeleton from implementing a control policy
// This node is provided to exemplify bringing together the
// encoder counts and ArUco detection measurements into one node,
// and using those measurement to set a motor duty cycle action.
//
// ----------------------------------------------------------------------------





// Include useful libraries
#include <math.h>
//#include <stdlib.h>
#include <vector>

// Include the ROS CPP packages
#include "ros/ros.h"
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"

// Include the asclinic message types
#include "asclinic_pkg/LeftRightFloat32.h"
#include "asclinic_pkg/LeftRightInt32.h"
#include "asclinic_pkg/FiducialMarkerArray.h"

// Include the asclinic constants
//#include "nodes/constant.h"

// Namespacing the package
//using namespace asclinic_pkg;





// MEMBER VARIABLES THAT ARE PARAMETERS FOR THIS NODE:
// > For the verbosity level of displaying info
//   Note: the levels of increasing verbosity are defined as:
//   0 : Info is not displayed. Warnings and errors are still displayed.
//   1 : Startup info is displayed.
//   2 : Info each control action and state estimate update is displayed.
int m_control_policy_verbosity = 1;

// > For the wheel base of the robot [in meters]
float m_robot_wheel_base = 0.22;

// > For the wheel radius of the robot [in meters]
float m_robot_wheel_radius = 0.072;

// > For the number of encoder counts per revolution of a wheel
int m_encoder_counts_per_wheel_revolution = 1680;



// ALL OTHER MEMBER VARIABLES FOR THIS NODE:
// > For world frame state estimate [xW, yW, phiW2R]
float m_xW_estimate = 0.0;
float m_yW_estimate = 0.0;
float m_phiW2R_estimate = 0.0;

// > For the rotation of the wheel per encoder count
float m_wheel_rotation_per_count = (2.0 * 3.14159265 / static_cast<float>(m_encoder_counts_per_wheel_revolution)) * m_robot_wheel_radius;

// > For the current direction of motor drive
float m_motor_left_drive_direction  = 0.0;
float m_motor_right_drive_direction = 0.0;

// > Publisher for the motor duty cycle requests
ros::Publisher m_motor_duty_cycle_request_publisher;





// Function that computes and publishes motor duty-cycle request actions
// baesd on the most recent state estimate
void execute_control_policy()
{
	// Sequnce number for tracking the order of published messages
	static unsigned long int s_sequence_number_for_msgs = 1;

	// Prepare a message to send the motor duty-cycle request action
	asclinic_pkg::LeftRightFloat32 msg_for_motors;
	msg_for_motors.left    = 0.0;
	msg_for_motors.right   = 0.0;
	msg_for_motors.seq_num = s_sequence_number_for_msgs;

	// Publish the message
	m_motor_duty_cycle_request_publisher.publish(msg_for_motors);

	// Increment the sequence number
	s_sequence_number_for_msgs++;
}



// Current motor duty cycle subscriber callback
void currentMotorDutyCycleSubscriberCallback(const asclinic_pkg::LeftRightFloat32& msg)
{
	// Display the values from the message received
	if (m_control_policy_verbosity >= 2)
		ROS_INFO_STREAM("[CONTROL POLICY SKELETON] Received current motor duty cycle (left,right,seq_num) = ( " << msg.left << " , " << msg.right << " , " << msg.seq_num << " )" );

	// Update the member variables to match the direction
	// of the current motor duty cycle.
	// > For the left wheel
	if (msg.left < -0.1)
		m_motor_left_drive_direction = -1.0;
	else if(msg.left > 0.1)
		m_motor_left_drive_direction = 1.0;
	else
		m_motor_left_drive_direction = 0.0;
	// > For the right wheel
	if (msg.right < -0.1)
		m_motor_right_drive_direction = -1.0;
	else if(msg.right > 0.1)
		m_motor_right_drive_direction = 1.0;
	else
		m_motor_right_drive_direction = 0.0;
}



// Encoder counts subscriber callback
// NOTE: this skeleton lets the receiving of the encoder counts
//       messages determine the frequency at which the control
//       policy runs.
void encoderCountsSubscriberCallback(const asclinic_pkg::LeftRightInt32& msg)
{
	// Display the values from the message received
	if (m_control_policy_verbosity >= 2)
		ROS_INFO_STREAM("[CONTROL POLICY SKELETON] Received encoder counts (left,right,seq_num) = ( " << msg.left << " , " << msg.right << " , " << msg.seq_num << " )" );

	// Compute the angular change of the wheels
	float delta_theta_left  = static_cast<float>(msg.left)  * m_motor_left_drive_direction  * m_wheel_rotation_per_count;
	float delta_theta_right = static_cast<float>(msg.right) * m_motor_right_drive_direction * m_wheel_rotation_per_count;

	// Compute the change in displacement (delta s) and change in rotation (delta phi) resulting from the wheel rotations
	float delta_s   = (delta_theta_right + delta_theta_left) * 0.5 * m_robot_wheel_radius;
	float delta_phi = (delta_theta_right - delta_theta_left) * 0.5 * m_robot_wheel_radius / m_robot_wheel_base;

	// Get the current state estimate in local variables to avoid errors/misinterpretation related to the sequence of updates
	float xW_current = m_xW_estimate;
	float yW_current = m_yW_estimate;
	float phiW2R_current = m_phiW2R_estimate;

	// Compute the sine and cosine of the "halfway" angle for the wheel odometry mean and covariance formulas
	float sin_phi_plus_half = sin(phiW2R_current + 0.5 * delta_phi);
	float cos_phi_plus_half = cos(phiW2R_current + 0.5 * delta_phi);

	// Update the mean of the state estimate
	m_xW_estimate = xW_current + delta_s * cos_phi_plus_half;
	m_yW_estimate = yW_current + delta_s * sin_phi_plus_half;
	m_phiW2R_estimate = phiW2R_current + delta_phi;

	// Update the covariance of the state estimate
	// > NOTE: this skeleton does not include co-variance

	// Call the function to execute the control policy
	execute_control_policy();
}



// ArUco Detections subscriber callback
// Details about the ArUco detection data:
// > The properties "rvec" and "tvec" respectively
//   describe the rotation and translation of the
//   marker frame relative to the camera frame, i.e.:
//   tvec - is a vector of length 3 expressing the
//          (x,y,z)-coordinates of the marker's center
//          in the coordinate frame of the camera.
//   rvec - is a vector of length 3 expressing the
//          rotation of the marker's frame relative to
//          the frame of the camera. This vector is an
//          "axis angle" representation of the rotation.
// > Hence, a vector expressed in maker-frame coordinates
//   can be transformed to camera-frame coordinates as:
//   - Rmat = cv2.Rodrigues(rvec)
//   - [x,y,z]_{in camera frame} = tvec + Rmat * [x,y,z]_{in marker frame}
// > Note: the camera frame convention is:
//   - z-axis points along the optical axis, i.e., straight out of the lens
//   - x-axis points to the right when looking out of the lens along the z-axis
//   - y-axis points to the down  when looking out of the lens along the z-axis
void arucoDetectionsSubscriberCallback(const asclinic_pkg::FiducialMarkerArray& msg)
{
	// Display the data received
	if (m_control_policy_verbosity >= 2)
		ROS_INFO_STREAM("[CONTROL POLICY SKELETON] Received aruco detections data for " << msg.num_markers << " markers." );

	// Iterate through the array of detected markers
	for(int i_marker=0; i_marker<msg.num_markers; i_marker++)
	{
		// Get the ID, tvec, and rvec for this marker
		float this_id   = msg.markers[i_marker].id;
		std::vector<float> this_tvec( msg.markers[i_marker].tvec.begin(), msg.markers[i_marker].tvec.end() );
		std::vector<float> this_rvec( msg.markers[i_marker].rvec.begin(), msg.markers[i_marker].rvec.end() );
	}
	
	// Convert to a World frame estimate
	// NOTE: this skeleton does not include this conversation

	// Fuse with the current state estimate
	// NOTE: this skeleton does not include sensor fusion
}



int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "control_policy_skeleton");
	ros::NodeHandle nodeHandle("~");

	// Initialise a node handle to the group namespace
	std::string ns_for_group = ros::this_node::getNamespace();
	ros::NodeHandle nh_for_group(ns_for_group);

	// Display that this node is launcing
	ROS_INFO_STREAM("[CONTROL POLICY SKELETON] Now launching this node in namespace: " << ns_for_group);

	// GET THE PARAMETER VALUES:
	// > For the verbosity level of displaying info
	if ( !nodeHandle.getParam("control_policy_verbosity", m_control_policy_verbosity) ) {
		ROS_WARN("[CONTROL POLICY SKELETON] FAILED to get \"control_policy_verbosity\" parameter. Using default value instead.");
	}
	// > For the wheel base dimension of the robot
	if ( !nodeHandle.getParam("robot_wheel_base", m_robot_wheel_base) ) {
		ROS_WARN("[CONTROL POLICY SKELETON] FAILED to get \"robot_wheel_base\" parameter. Using default value instead.");
	}
	// > For the wheel radius dimension of the robot
	if ( !nodeHandle.getParam("robot_wheel_radius", m_robot_wheel_radius) ) {
		ROS_WARN("[CONTROL POLICY SKELETON] FAILED to get \"robot_wheel_radius\" parameter. Using default value instead.");
	}
	// > For the number of encoder counts per revolution of a wheel
	if ( !nodeHandle.getParam("encoder_counts_per_wheel_revolution", m_encoder_counts_per_wheel_revolution) ) {
		ROS_WARN("[CONTROL POLICY SKELETON] FAILED to get \"encoder_counts_per_wheel_revolution\" parameter. Using default value instead.");
	}

	// > Compute the value of wheel rotation per encoder count,
	//   based on the parameter values retrieved
    m_wheel_rotation_per_count = (2.0 * 3.14159265 / static_cast<float>(m_encoder_counts_per_wheel_revolution)) * m_robot_wheel_radius;



	// PUBLISHERS AND SUBSCRIBERS:
	// > Initialise a publisher for the motor duty cycle requests
	m_motor_duty_cycle_request_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightFloat32>("set_motor_duty_cycle", 1, true);

	// > Initialise a subscriber for the current motor duty cycle
	ros::Subscriber current_motor_duty_cycle_subscriber = nh_for_group.subscribe("current_motor_duty_cycle", 1, currentMotorDutyCycleSubscriberCallback);

	// > Initialise a subscriber for the encoder counts sensor measurements
	ros::Subscriber encoder_counts_subscriber = nh_for_group.subscribe("encoder_counts", 10, encoderCountsSubscriberCallback);

	// > Initialise a subscriber for the ArUco marker vision-detection measurements
	ros::Subscriber aruco_detections_subscriber = nh_for_group.subscribe("aruco_detections", 10, arucoDetectionsSubscriberCallback);



	// Display the status
	if (m_control_policy_verbosity >= 1)
		ROS_INFO("[CONTROL POLICY SKELETON] Node initialisation complete.");



	// Spin as a single-threaded node
	ros::spin();

	// Release any hardware resources that were opened
	// > Nothing to release
	
	return 0;
}
