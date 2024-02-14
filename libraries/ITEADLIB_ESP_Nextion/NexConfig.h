/**
 * @file NexConfig.h
 *
 * Options for user can be found here. 
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/13
 * @author Jyrki Berg 2/17/2019 (https://github.com/jyberg)
 * 
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 * 
 * @copyright 2020 Jyrki Berg
 *
 */

#pragma once

/**
 * @addtogroup Configuration 
 * @{ 
 */

/**
 * Define serial communication default baud.
 * it is recommended that do not change defaul baud on Nextion, because it can forgot it on re-start
 * If changing this value check that vakue is the same as factory set default baud in the used display.
 * 
*/
#define NEX_SERIAL_DEFAULT_BAUD 9600

/**
 * Define standard (dafault) or fast timeout,  you may use fast timeout in case of baudrate higher than 115200
 * 
*/
#define NEX_TIMEOUT_STANDARD
//#define NEX_TIMEOUT_FAST

#ifdef NEX_TIMEOUT_FAST
#define NEX_TIMEOUT_COMMAND   10
#define NEX_TIMEOUT_RETURN    10
#else
#define NEX_TIMEOUT_COMMAND  200
#define NEX_TIMEOUT_RETURN   100
#endif

#define NEX_TIMEOUT_TRANSPARENT_DATA_MODE 400


/** 
 * Define DEBUG_SERIAL_ENABLE to enable debug serial. 
 * Comment it to disable debug serial. 
 */
//#define DEBUG_SERIAL_ENABLE

/**
 * Define dbSerial for the output of debug messages. 
 * it is resonsibility of main program to initialize debug serial port (begin(...)
 */
//#define dbSerial Serial

// Enable Next TFT file upload functionality
//#define NEX_ENABLE_TFT_UPLOAD

// Enable HardwareSerial support by definign NEX_ENABLE_HW_SERIAL
#define NEX_ENABLE_HW_SERIAL

// Enable SoftwareSerial support by definign NEX_ENABLE_SW_SERIAL
// NodeMcu / Esp8266 use Softwareserial if usb port is used for debuging
// NodeMcu board pin numbers not match with Esp8266 pin numbers use NodeMcu Pin number definitions (pins_arduino.h)
#define NEX_ENABLE_SW_SERIAL


#ifdef DEBUG_SERIAL_ENABLE
#define dbSerialPrint(a)    dbSerial.print(a)
#define dbSerialPrintln(a)  dbSerial.println(a)
#define dbSerialBegin(a)    dbSerial.begin(a)
#else
#define dbSerialPrint(a)   do{}while(0)
#define dbSerialPrintln(a) do{}while(0)
#define dbSerialBegin(a)   do{}while(0)
#endif

/**
 * @}
 */
