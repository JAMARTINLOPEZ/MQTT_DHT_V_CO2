/**
 * @file NexGauge.h
 *
 * The definition of class NexGauge. 
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
 * NexGauge component.
 */
class NexGauge: public NexObject
{
    NexGauge()=delete;

public: /* methods */
    /**
     * @copydoc NexObject::NexObject(Nextion*, uint8_t, uint8_t, const char*, const NexObject*)
     */
    NexGauge(Nextion *nextion, uint8_t pid, uint8_t cid, const char *name, const NexObject* page=nullptr);

    /**
     * Get the value of gauge.
     * 
     * @param number - an output parameter to save gauge's value.
     * 
     * @return true if success, false for failure
     */
    bool getValue(uint32_t *number);
    
    /**
     * Set the value of gauge. 
     *
     * @param number - the value of gauge.  
     *
     * @return true if success, false for failure
     */
    bool setValue(uint32_t number);
	
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

    /**
     * Get wid attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure 
     */	
    bool Get_pointer_thickness_wid(uint32_t *number);		
	
    /**
     * Set wid attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_pointer_thickness_wid(uint32_t number);		

    /**
     * Get picc attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure 
     */
    bool Get_background_cropi_picc(uint32_t *number);		
	
    /**
     * Set picc attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_background_crop_picc(uint32_t number);			
};

/**
 * @}
 */
