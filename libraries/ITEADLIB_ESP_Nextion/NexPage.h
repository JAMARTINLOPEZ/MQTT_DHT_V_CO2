/**
 * @file NexPage.h
 *
 * The definition of class NexPage. 
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
 * A special component , which can contain other components such as NexButton, 
 * NexText and NexWaveform, etc. 
 */
class NexPage: public NexTouch
{
    NexPage()=delete;
    
public: /* methods */

    /**
     * Constructor. 
     *
     * @param nextion - nextion interface
     * @param pid - page id. 
     * @param name - pointer to an unique name in range of all components.
     */
    NexPage(Nextion *nextion, uint8_t pid, const char *name);
    
    /**
     * Show itself. 
     * 
     * @return true if success, false for faileure.
     */
    bool show(void);

    /**
     * Hide or Show all componens on current page
     *
     * @param visible - true Show components, false Hide components
     * @return true if success, false for failure
     */
    bool setVisibleAll(bool visible);
};
/**
 * @}
 */