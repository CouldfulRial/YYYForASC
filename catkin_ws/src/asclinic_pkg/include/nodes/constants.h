// Copyright (C) 2024, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
//
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
// This header defines constants for all the nodes
//
// ----------------------------------------------------------------------------



// ----------------------------------------------------------------------------
// VERBOSITY FLAGS
// ----------------------------------------------------------------------------
// Note: the levels of increasing verbosity are defined as:
// 0 : Info is not displayed. Warnings and errors are still displayed
// 1 : Startup info is displayed
// 2 : Info about messages received is displayed

const int DEFAULT_ENCODER_READ_VERBOSITY = 1;
const int DEFAULT_MOTOR_DRIVER_VERBOSITY = 1;
const int DEFAULT_SERVO_DRIVER_VERBOSITY = 1;
//const int DEFAULT_ARUCO_DETECTOR_VERBOSITY = 1;



// ----------------------------------------------------------------------------
// ENCODER COUNTING NODE CONSTANTS
// ----------------------------------------------------------------------------
const int DEFAULT_GPIO_CHIP_NUMBER_FOR_ENCODER_LINES = 1;

const bool DEFAULT_SHOULD_MONITOR_LEFT_SIDE_ENCODER_CHANNEL_A  = true;
const bool DEFAULT_SHOULD_MONITOR_LEFT_SIDE_ENCODER_CHANNEL_B  = false;
const bool DEFAULT_SHOULD_MONITOR_RIGHT_SIDE_ENCODER_CHANNEL_A = true;
const bool DEFAULT_SHOULD_MONITOR_RIGHT_SIDE_ENCODER_CHANNEL_B = false;

const int DEFAULT_LINE_NUMBER_FOR_LEFT_SIDE_ENCODER_CHANNEL_A  = 105;
const int DEFAULT_LINE_NUMBER_FOR_LEFT_SIDE_ENCODER_CHANNEL_B  = 106;
const int DEFAULT_LINE_NUMBER_FOR_RIGHT_SIDE_ENCODER_CHANNEL_A = 84;
const int DEFAULT_LINE_NUMBER_FOR_RIGHT_SIDE_ENCODER_CHANNEL_B = 130;

const string DEFAULT_ENCODER_EVENTS_TO_MONITOR = "rising"; // OPTIONS: "rising", "falling", "both"

const float DEFAULT_DELTA_T_FOR_PUBLISHING_ENCODER_COUNTS = 0.1;



// ----------------------------------------------------------------------------
// MOTOR DRIVER CONSTANTS
// ----------------------------------------------------------------------------
const int   DEFAULT_MOTOR_DRIVER_CURRENT_LIMIT_IN_MILLIAMPS = 5000;
const float DEFAULT_MOTOR_DRIVER_MAX_DUTY_CYCLE_LIMIT_IN_PERCENT = 100.0;
const float DEFAULT_MOTOR_DRIVER_MAX_ACCELERATION_LIMIT_IN_PERCENT_PER_MILLISECOND = 0.04;
const float DEFAULT_MOTOR_DRIVER_MAX_DECELERATION_LIMIT_IN_PERCENT_PER_MILLISECOND = 0.16;

const float DEFAULT_MOTOR_DRIVER_LEFT_SIDE_MULTIPLIER  = 1.0;
const float DEFAULT_MOTOR_DRIVER_RIGHT_SIDE_MULTIPLIER = 1.0;



// ----------------------------------------------------------------------------
// SERVO DRIVER CONSTANTS
// ----------------------------------------------------------------------------
const float    DEFAULT_SERVO_DRIVER_PWM_FREQUENCY_IN_HERTZ = 50.0;
const uint16_t DEFAULT_SERVO_DRIVER_MIN_PULSE_WIDTH_IN_MICROSECONDS = 500;
const uint16_t DEFAULT_SERVO_DRIVER_MAX_PULSE_WIDTH_IN_MICROSECONDS = 2500;
