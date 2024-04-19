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
// Constants for the I2C driver for the INA260 {current,voltage,power} sensor
//
// ----------------------------------------------------------------------------





#ifndef INA260_CONSTANTS_H
#define INA260_CONSTANTS_H



// I2C ADDRESS
#define INA260_I2C_ADDRESS_DEFAULT     0x40


// REGISTERS
#define INA260_REGISTER_CONFIG                      0x00
#define INA260_REGISTER_CURRENT_MEASUREMENT         0x01
#define INA260_REGISTER_BUS_VOLTAGE_MEASUREMENT     0x02
#define INA260_REGISTER_POWER_CALCULATION           0x03
#define INA260_REGISTER_MASK_ENABLE                 0x06
#define INA260_REGISTER_ALERT_LIMIT_VALUE           0x07
#define INA260_REGISTER_MANUFACTURER_UID            0xFE
#define INA260_REGISTER_DIE_AND_REVISION_UID        0xFF



// LEAST-SIGNIFICANT-BIT (LSB) UNITS
#define INA260_CURRENT_REGISTER_LSB_UNITS_MILLIAMPS       1.25
#define INA260_VOLTAGE_REGISTER_LSB_UNITS_MILLIVOLTS      1.25
#define INA260_POWER_REGISTER_LSB_UNITS_MILLIWATTS       10.00



#endif // INA260_CONSTANTS_H
