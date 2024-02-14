/**
* @file NexScreen.h
*
* The definition of class NexScreen.
*
* @author Francesco Doni 8/1/2022 (https://github.com/frdoni)
* @date 1/8/2022
*
* @copyright 
* Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
* 
* @copyright 2022 Francesco Doni
*
*/

#pragma once

#include "NexTouch.h"

class Nextion;
class NexObject;

/**
 * @addtogroup Component 
 * @{ 
 */
 
/**
* NexScreen component.
*
*
*/
class NexScreen : public NexTouch
{
	NexScreen()=delete;
	
public: /* methods */

	/**
     * @copydoc NexObject::NexObject(Nextion*, uint8_t, uint8_t, const char*, const NexObject*)
     */
	NexScreen(Nextion *nextion);
	
	/**
	* Set the brightness of the display.
	*
	* @param number (0 - 100).
	* @return true if success, false for failure 
	*/
	bool setBacklightLevel(uint32_t number);
	
	/**
	* Set Nextion sleep mode.
	*
	* @param none
	* @return true if success, false for failure
	*/
	bool invokeScreenSleep();
	
	/**
	* Set Nextion awake mode.
	*
	* @param none
	* @return true if success, false for failure
	*/
	bool invokeScreenWakeup();
	
	/**
	* Set Auto Wake on Touch.
	*
	* @param number, 0 (do not wake), 1 (wake on touch), default 0.
	* @return true if success, false for failure
	*/
	
	bool setScreenAutoWakeup(uint32_t number);
	
	/**
	* Set Sleep on No Touch.
	*
	* @param number of second, 3 - 65535, default 0 (means no timeout)
	* @return true if success, false for failure
	*/
	bool setScreenTouchTimeout(uint32_t number);
	
	/**
	* Sets internal No-serial-then-sleep timer to specified value in seconds.
	*
	* @param number of second, 3 - 65535, default 0 (means no timeout)
	* @return true if success, false for failure
	*/
	bool setSleepOnNoSerial(uint32_t number);
	
	/**
	* Sets if serial data wakes Nextion from sleep mode automatically.
	*
	* @param number, min is 0, max is 1, default 0
	* @return true if success, false for failure
	*/
	bool setWakeOnSerialData(uint32_t number);
};
/**
* @}
*/

