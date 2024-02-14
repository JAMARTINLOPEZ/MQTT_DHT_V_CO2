/**
 * @file NexProgressBar.h
 *
 * The definition of class NexProgressBar. 
 *
 * @author Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date 2015/8/13
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

#include "NexObject.h"

class Nextion;

/**
 * @addtogroup Component 
 * @{ 
 */

/**
 * NexProgressBar component. 
 */
class NexProgressBar: public NexObject
{
    NexProgressBar()=delete;

public: /* methods */
    /**
     * @copydoc NexObject::NexObject(Nextion*, uint8_t, uint8_t, const char*, const NexObject*)
     */
    NexProgressBar(Nextion *nextion, uint8_t pid, uint8_t cid, const char *name, const NexObject* page=nullptr);
    
    /**
     * Get the value of progress bar. 
     * 
     * @param number - an output parameter to save the value of porgress bar.  
     * 
     * @return true if success, false for failure
     */
    bool getValue(uint32_t *number);
    
    /**
     * Set the value of progress bar.
     *
     * @param number - the value of progress bar.  
     *
     * @return true if success, false for failure
     */
    bool setValue(uint32_t number);

    /**
     * Set the value of background image (bpic)
     *
     * @param number - the value of the background image
     *
     * @return true if success, false for failure
     */
    bool set_background_picture(uint32_t number);

    /**
     * Set the value of foreground image (ppic)
     *
     * @param number - the value of the foreground image
     *
     * @return true if success, false for failure
     */
    bool set_foreground_picture(uint32_t number);

    /**
     * Get bco attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure 
     */
    bool Get_background_color_bco(uint32_t *number);
	
    /**
     * Set bco attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_background_color_bco(uint32_t number);
	
    /**
     * Get pco attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure 
     */
    bool Get_font_color_pco(uint32_t *number);		
	
    /**
     * Set pco attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_font_color_pco(uint32_t number);			
};

/**
 * @}
 */
