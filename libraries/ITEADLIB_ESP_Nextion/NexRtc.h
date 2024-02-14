/**
 * @file NexRtc.h
 *
 * The definition of class NexRtc. 
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

#include "NextionIf.h"

class Nextion;

/**
 * @addtogroup Component 
 * @{ 
 */

/**
 * NexRtc component.
 */
class NexRtc : public NextionIf
{
    NexRtc()=delete;

    public:

    /**
     * Constructor. 
     *
     * @param nextion - nextion interface
     */
    NexRtc(Nextion *nextion);

    ~NexRtc();

    /**
     * write rtc times
     *
     * @param time - time string
     * @return true if success, false for failure
     */   
    bool write_rtc_time(char *time);
    
    /**
     * write rtc times
     *
     * @param time_type - To type in time   (example:write_rtc_time("year",2016))
     * @param number - the time value
     * @return true if success, false for failure
     */   
    bool write_rtc_time(char *time_type,uint32_t number);
    
    /**
     * write rtc times
     *
     * @param time - Time to write to the array
     * @return true if success, false for failure
     */
    bool write_rtc_time(uint32_t *time);
    
    
    /**
     * read rtc time
     *
     * @param time - Access data array
     * @param len - len of array
     * @return true if success, false for failure
     */
    bool read_rtc_time(char *time,uint32_t len);
    
    /**
     * read rtc times
     *
     * @param time_type - To type in time   
     * @param number - the time value
     * @return true if success, false for failure
     */
    bool read_rtc_time(char *time_type,uint32_t *number);
    
    /**
     * read rtc time
     *
     * @param time - Access data array
     * @param len - len of array
     * @return true if success, false for failure
     */
    bool read_rtc_time(uint32_t *time,uint32_t len);
    
};

/**
 * @}
 */