/**
 * @file NextionIf.h
 *
 * Nextion interface implementation interface
 *
 * @author Jyrki Berg 11/23/2019 (https://github.com/jyberg)
 * 
 * @copyright 2020 Jyrki Berg
 *
 */

#pragma once
#include "NexConfig.h"
#include "NexHardwareInterface.h"
class Nextion;


/**
 * @addtogroup CoreAPI 
 * @{ 
 */

/**
 * Nextion Interface class implementation for Nextion objects
 *
 * Provides call wrapper to nextion interface
 */
class NextionIf:public NextionInterface
{
public:

    NextionIf(Nextion *nextion);
    ~NextionIf();

/* Receive unsigned number
*
* @param number - received value
* @param timeout - set timeout time.
*
* @retval true - success.
* @retval false - failed. 
*/
bool recvRetNumber(uint32_t *number, size_t timeout = NEX_TIMEOUT_RETURN) final;

/* Receive signed number
*
* @param number - received value
* @param timeout - set timeout time.
*
* @retval true - success.
* @retval false - failed. 
*/
bool recvRetNumber(int32_t *number, size_t timeout = NEX_TIMEOUT_RETURN) final;

/* Receive string
*
* @param string - received value
* @param timeout - set timeout time.
* @param start_flag - is str start flag (0x70) expected, default falue true
*
* @retval true - success.
* @retval false - failed. 
*/
virtual bool recvRetString(String &str, size_t timeout = NEX_TIMEOUT_RETURN, bool start_flag = true) final;

/* Receive string
*
* @param buffer - received value buffer
* @param le - value buffer size
* @param timeout - set timeout time.
* @param start_flag - is str start flag (0x70) expected, default falue true
*
* @retval true - success.
* @retval false - failed. 
*/
bool recvRetString(char *buffer, uint16_t &len, size_t timeout = NEX_TIMEOUT_RETURN, bool start_flag = true) final;

/* Send Command to device
*
* parameter command string
*/
void sendCommand(const char* cmd) final;

/* Send Raw data to device
*
* parameter raw data buffer
*/
#ifdef ESP8266
void sendRawData(const std::vector<uint8_t> &data) final;
#endif

/* Send Raw data to device
*
* @param buf - raw data buffer poiter
* @param len - raw data buffer pointer
*/
void sendRawData(const uint8_t *buf, uint16_t len) final;


/* Send Raw byte to device
*
* parameter raw byte
*/
void sendRawByte(const uint8_t byte) final;

/* read Bytes from device
 * @brief 
 * 
 * @param buffer - receive buffer
 * @param size  - bytes to read
 * @param timeout  timeout ms
 * @return size_t read bytes can be less that size (timeout case) 
 */
size_t readBytes(uint8_t* buffer, size_t size, size_t timeout =NEX_TIMEOUT_RETURN) final;

/* Receive command
*
* @param command - command to be received / checked
* @param timeout - set timeout time.
*
* @retval true - success.
* @retval false - failed. 
*/
bool recvCommand(const uint8_t command, size_t timeout) final;

/*
 * Command is executed successfully. 
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed. 
 *
 */
bool recvRetCommandFinished(size_t timeout = NEX_TIMEOUT_COMMAND) final;

/*
 * Transpared data mode setup successfully 
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed. 
 *
 */
bool RecvTransparendDataModeReady(size_t timeout = NEX_TIMEOUT_TRANSPARENT_DATA_MODE) final;

/*
 * Transpared data mode finished 
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed. 
 *
 */
bool RecvTransparendDataModeFinished(size_t timeout = NEX_TIMEOUT_COMMAND) final;

/**
 * current baud value
 * 
 * 
 * @return current baud value
 */
uint32_t GetCurrentBaud() final;


private: // data
    Nextion *m_nextion; // nextion interface instance
};

/**
 * @} 
 */
