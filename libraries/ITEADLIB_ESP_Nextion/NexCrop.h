/**
 * @file NexCrop.h
 *
 * The definition of class NexCrop. 
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

#include "NexTouch.h"

class Nextion;
class NexObject;

/**
 * @addtogroup Component 
 * @{ 
 */

/**
 * NexCrop component. 
 */
class NexCrop: public NexTouch
{
    NexCrop()=delete;

public: /* methods */

    /**
     * @copydoc NexObject::NexObject(Nextion*, uint8_t, uint8_t, const char*, const NexObject*)
     */
    NexCrop(Nextion *nextion, uint8_t pid, uint8_t cid, const char *name, const NexObject* page=nullptr);

    /**
     * Get the number of picture. 
     *
     * @param number - an output parameter to save the number of picture. 
     * 
     * @return true if success, false for failure
     */
    bool Get_background_crop_picc(uint32_t *number);
    
    /**
     * Set the number of picture. 
     *
     * @param number - the number of picture. 
     * 
     * @return true if success, false for failure
     */
    bool Set_background_crop_picc(uint32_t number);
	
	/**
     * Get the number of picture. 
     *
     * @param number - an output parameter to save the number of picture. 
     * 
     * @return true if success, false for failure
     */
    bool getPic(uint32_t *number);
    
    /**
     * Set the number of picture. 
     *
     * @param number - the number of picture. 
     * 
     * @return true if success, false for failure
     */
    bool setPic(uint32_t number);
};

/**
 * @}
 */