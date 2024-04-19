// Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
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
// I2C driver for the Pololu Simple Motor Controller (SMC) G2
//
// ----------------------------------------------------------------------------





#ifndef POLOLU_SMC_G2_CONSTANTS_H
#define POLOLU_SMC_G2_CONSTANTS_H



// I2C ADDRESS
#define POLOLU_SMC_G2_I2C_ADDRESS_DEFAULT     0x0D



// COMMANDS
#define POLOLU_SMC_G2_COMMAND_EXIT_SAFE_START          0x83
#define POLOLU_SMC_G2_COMMAND_MOTOR_FORWARD            0x85
#define POLOLU_SMC_G2_COMMAND_MOTOR_REVERSE            0x86
#define POLOLU_SMC_G2_COMMAND_MOTOR_FORWARD_7BIT       0x89
#define POLOLU_SMC_G2_COMMAND_MOTOR_REVERSE_7BIT       0x8A
#define POLOLU_SMC_G2_COMMAND_MOTOR_BRAKE              0x92
#define POLOLU_SMC_G2_COMMAND_GET_VARIABLE             0xA1
#define POLOLU_SMC_G2_COMMAND_SET_MOTOR_LIMIT          0xA2
#define POLOLU_SMC_G2_COMMAND_SET_CURRENT_LIMIT        0x91
#define POLOLU_SMC_G2_COMMAND_GET_FIRMWARE_VERSION     0xC2
#define POLOLU_SMC_G2_COMMAND_STOP_MOTOR               0xE0




// SET MOTOR LIMITS IDs
// > For setting forward and reverse limits at the same time
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_SPEED                       0
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_ACCELERATION                1
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_DECELERATION                2
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_BRAKE_DURATION              3
// > For setting forward limits
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_SPEED_FORWARD               4
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_ACCELERATION_FORWARD        5
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_DECELERATION_FORWARD        6
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_BRAKE_DURATION_FORWARD      7
// > For setting reverse limits
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_SPEED_REVERSE               8
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_ACCELERATION_REVERSE        9
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_DECELERATION_REVERSE       10
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_BRAKE_DURATION_REVERSE     11


// SET MOTOR LIMITS RESPONSE CODES
// > For the numerical code
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_RESPONSE_CODE_0     0
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_RESPONSE_CODE_1     1
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_RESPONSE_CODE_2     2
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_RESPONSE_CODE_3     3
// > For the description of the code
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_RESPONSE_CODE_0_STRING     "No problems setting the limit"
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_RESPONSE_CODE_1_STRING     "Unable to set forward limit to the specified value because of hard motor limit settings."
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_RESPONSE_CODE_2_STRING     "Unable to set reverse limit to the specified value because of hard motor limit settings."
#define POLOLU_SMC_G2_SET_MOTOR_LIMIT_RESPONSE_CODE_3_STRING     "Unable to set forward and reverse limits to the specified value because of hard motor limit settings."





// GET VARIABLE IDs
// > For the status flag registers
#define POLOLU_SMC_G2_GET_VARIABLE_ERROR_STATUS                   0
#define POLOLU_SMC_G2_GET_VARIABLE_ERRORS_OCCURRED                1
#define POLOLU_SMC_G2_GET_VARIABLE_SERIAL_ERRORS_OCCURRED         2
#define POLOLU_SMC_G2_GET_VARIABLE_LIMIT_STATUS                   3
#define POLOLU_SMC_G2_GET_VARIABLE_RESET_FLAGS                  127
// > For the RC channels
#define POLOLU_SMC_G2_GET_VARIABLE_RC1_UNLIMITED_RAW_VALUE        4
#define POLOLU_SMC_G2_GET_VARIABLE_RC1_RAW_VALUE                  5
#define POLOLU_SMC_G2_GET_VARIABLE_RC1_SCALED_VALUE               6
#define POLOLU_SMC_G2_GET_VARIABLE_RC2_UNLIMITED_RAW_VALUE        8
#define POLOLU_SMC_G2_GET_VARIABLE_RC2_RAW_VALUE                  9
#define POLOLU_SMC_G2_GET_VARIABLE_RC2_SCALED_VALUE              10
// > For the analog channel inputs
#define POLOLU_SMC_G2_GET_VARIABLE_AN1_UNLIMITED_RAW_VALUE       12
#define POLOLU_SMC_G2_GET_VARIABLE_AN1_RAW_VALUE                 13
#define POLOLU_SMC_G2_GET_VARIABLE_AN1_SCALED_VALUE              14
#define POLOLU_SMC_G2_GET_VARIABLE_AN2_UNLIMITED_RAW_VALUE       16
#define POLOLU_SMC_G2_GET_VARIABLE_AN2_RAW_VALUE                 17
#define POLOLU_SMC_G2_GET_VARIABLE_AN2_SCALED_VALUE              18
// > For diagnostic variables
#define POLOLU_SMC_G2_GET_VARIABLE_TARGET_SPEED                  20
#define POLOLU_SMC_G2_GET_VARIABLE_SPEED                         21
#define POLOLU_SMC_G2_GET_VARIABLE_BRAKE_AMOUNT                  22
#define POLOLU_SMC_G2_GET_VARIABLE_INPUT_VOLTAGE                 23
#define POLOLU_SMC_G2_GET_VARIABLE_TEMPERATURE_A                 24
#define POLOLU_SMC_G2_GET_VARIABLE_TEMPERATURE_B                 25
#define POLOLU_SMC_G2_GET_VARIABLE_RC_PERIOD                     26
#define POLOLU_SMC_G2_GET_VARIABLE_BAUD_RATE_REGISTER            27
#define POLOLU_SMC_G2_GET_VARIABLE_UP_TIME_LOW                   28
#define POLOLU_SMC_G2_GET_VARIABLE_UP_TIME_HIGH                  29
// > For motor speed limits (forward)
#define POLOLU_SMC_G2_GET_VARIABLE_MAX_SPEED_FORWARD             30
#define POLOLU_SMC_G2_GET_VARIABLE_MAX_ACCELERATION_FORWARD      31
#define POLOLU_SMC_G2_GET_VARIABLE_MAX_DECELERATION_FORWARD      32
#define POLOLU_SMC_G2_GET_VARIABLE_BRAKE_DURATION_FORWARD        33
#define POLOLU_SMC_G2_GET_VARIABLE_STARTING_SPEED_FORWARD        34
// > For motor speed limits (reverse)
#define POLOLU_SMC_G2_GET_VARIABLE_MAX_SPEED_REVERSE             36
#define POLOLU_SMC_G2_GET_VARIABLE_MAX_ACCELERATION_REVERSE      37
#define POLOLU_SMC_G2_GET_VARIABLE_MAX_DECELERATION_REVERSE      38
#define POLOLU_SMC_G2_GET_VARIABLE_BRAKE_DURATION_REVERSE        39
#define POLOLU_SMC_G2_GET_VARIABLE_STARTING_SPEED_REVERSE        40
// > For current limiting and measurement
#define POLOLU_SMC_G2_GET_VARIABLE_CURRENT_LIMIT                 42
#define POLOLU_SMC_G2_GET_VARIABLE_RAW_CURRENT                   43
#define POLOLU_SMC_G2_GET_VARIABLE_CURRENT                       44
#define POLOLU_SMC_G2_GET_VARIABLE_CURRENT_LIMITING_CONSECUTIVE_COUNT     45
#define POLOLU_SMC_G2_GET_VARIABLE_CURRENT_LIMITING_OCCURRENCE_COUNT      46





#endif // POLOLU_SMC_G2_CONSTANTS_H
