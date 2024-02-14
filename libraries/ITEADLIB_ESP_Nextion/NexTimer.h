/**
 * @file NexTimer.h
 *
 * The definition of class NexTimer. 
 *
 * @author huang xianming (email:<xianming.huang@itead.cc>)
 * @date 2015/8/26
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
 * NexTimer component. 
 *
 * Commonly, you want to do something after set timer cycle and enable it,and the cycle value 
 * must be greater than 50
 * 
 */
class NexTimer: public NexTouch
{
    NexTimer()=delete;

public: /* methods */

    /**
     * @copydoc NexObject::NexObject(Nextion*, uint8_t, uint8_t, const char*, const NexObject*)
     */
    NexTimer(Nextion *nextion, uint8_t pid, uint8_t cid, const char *name, const NexObject* page=nullptr);

    /**
     * Attach an callback function of timer respond event. 
     *
     * @param timer - callback called with ptr when a timer respond event occurs. 
     * @param ptr - parameter passed into push[default:nullptr]. 
     * @return none. 
     *
     * @note If calling this method multiply, the last call is valid. 
     */
    void attachTimer(NexTouchEventCb timer, void *ptr = nullptr);

    /**
     * Detach an callback function. 
     * 
     * @return none. 
     */
    void detachTimer(void);

    /**
     * Get the value of timer cycle val. 
     * 
     * @param number - an output parameter to save the value of timer cycle.  
     * 
     * @return true if success, false for failure
     */
    bool getCycle(uint32_t *number); 

    /**
     * Set the value of timer cycle val.
     *
     * @param number - the value of timer cycle.  
     * 
     * @return true if success, false for failure
     *
     * @warning  the cycle value must be greater than 50.
     */
    bool setCycle(uint32_t number);   

    /**
     * contorl timer enable.
     *
     * @return true if success, false for failure
     */
    bool enable(void);

    /**
     * contorl timer disable.
     *
     * @return true if success, false for failure
     */
    bool disable(void); 
    
    /**
     * Get tim attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure 
     */
    bool Get_cycle_tim(uint32_t *number);

    /**
     * Set tim attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_cycle_tim(uint32_t number);
};
/**
 * @}
 */
