/**
 * @file NexWaveform.h
 *
 * The definition of class NexWaveform. 
 *
 * @author Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date 2015/8/13
 * @author Jyrki Berg 3/3/2019 (https://github.com/jyberg)
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

#ifdef ESP8266
#include <vector>
#endif

#include "NexTouch.h"

class Nextion;
class NexObject;

/**
 * @addtogroup Component 
 * @{ 
 */

/**
 * NexWaveform component.
 */
class NexWaveform: public NexTouch
{
    NexWaveform()=delete;

public: /* methods */
    /**
     * @copydoc NexObject::NexObject(Nextion*, uint8_t, uint8_t, const char*, const NexObject*)
     */
    NexWaveform(Nextion *nextion, uint8_t pid, uint8_t cid, const char *name, const NexObject* page=nullptr);

     /**
     * Constructor with value scaling parameters, Scales added value to set Waveform scale 
     *
     * @param nextion - nextion interface
     * @param pid - page id. 
     * @param cid - component id.    
     * @param name - pointer to an unique name in range of all components.
     * @param minVal - scaling minimum value
     * @param maxVal - scaling maximum value
     * @param hight - componetn hight
     * @param page - pointer to global page information (can be nullptr in case local object)
     */
    NexWaveform(Nextion *nextion, uint8_t pid, uint8_t cid, const char *name, 
        float minVal, float maxVal, uint8_t hight,
        const NexObject* page=nullptr);

    /**
     * Add value to show. 
     *
     * @tparam T - typename (numeeric tupes supported)
     * @param ch - channel of waveform(0-3). 
     * @param value - the value of waveform (value is scaled to nextion resolution 0-255 based on Min / Max component hight).  
     *
     * @return true if success, false for failure
     */
    template<typename T>
    bool addValue(uint8_t ch, T value)
    {
        #ifdef ESP8266
        // compile time data type check 
        static_assert(std::is_arithmetic<T>::value, "Not numeric type");
        #endif
        char buf[15] = {0};
        
        if (ch > 3)
        {
            return false;
        }

        sprintf(buf, "add %u,%u,%u", getObjCid(), ch, ScaleToForm(value));
        sendCommand(buf);
        return true;
    }

#ifdef ESP8266
    /**
     * Add values to show. 
     *
     * @tparam T - typename (numeeric tupes supported)
     * @param ch - channel of waveform(0-3). 
     * @param values - the values of waveform (values are scaled to nextion resolution 0-255 based on Min / Max component hight).  
     *
     * @return true if success, false for failure
     */
    template<typename T>
    bool addValues(uint8_t ch, std::vector<T> &values)
    {
        // compile time data type check  
        static_assert(std::is_arithmetic<T>::value, "Not numeric type");

        bool ret=true;
 
        for(uint16_t offset{0}; offset < values.size() && ret;)
        {
            uint32_t sendBytes{values.size()-offset};
            if(sendBytes>124)
            {
                sendBytes=124;
            }
            { 
            char buf[4] = {0};
            utoa(getObjCid(), buf, 10);
            String cmd;
            cmd = "addt ";
            cmd += buf;
            cmd += ",";
            utoa(ch, buf, 10);
            cmd += buf;
            cmd += ",";
            utoa(sendBytes, buf, 10);
            cmd += buf;
            sendCommand(cmd.c_str());
            }

            if(!RecvTransparendDataModeReady())
            {
                ret=false;
                break;
            }
            for(uint16_t i{0}; i<sendBytes; ++i)
            {
                sendRawByte(ScaleToForm(values[offset++]));
            }
            if(!RecvTransparendDataModeFinished())
            {
                ret=false;
            }
        }
 
        return ret;
    }
#endif

    /**
     * Add values to show. 
     * 
     * @tparam T - typename (numeeric tupes supported)
     * @param ch - channel of waveform(0-3). 
     * @param values - pointer to the values of waveform (values are scaled to nextion resolution 0-255 based on Min / Max component hight). 
     * @param len - values buffer len
     *
     * @retval true - success false - failed. 
     */
    template<typename T>
    bool addValues(uint8_t ch, T *values, uint16_t len)
    {
        #ifdef ESP8266
        // compile time data type check 
        static_assert(std::is_arithmetic<T>::value, "Not numeric type");
        #endif

        bool ret=true;
 
        for(uint16_t offset{0}; offset < len && ret;)
        {
            uint32_t sendBytes{len-offset};
            if(sendBytes>124)
            {
                sendBytes=124;
            }
            { 
            char buf[4] = {0};
            utoa(getObjCid(), buf, 10);
            String cmd;
            cmd = "addt ";
            cmd += buf;
            cmd += ",";
            utoa(ch, buf, 10);
            cmd += buf;
            cmd += ",";
            utoa(sendBytes, buf, 10);
            cmd += buf;
            sendCommand(cmd.c_str());
            }

            if(!RecvTransparendDataModeReady())
            {
                ret=false;
                break;
            }
            for(uint16_t i{0}; i<sendBytes; ++i)
            {
                sendRawByte(ScaleToForm(values[offset++]));
            }
            if(!RecvTransparendDataModeFinished())
            {
                ret=false;
            }
        }
 
        return ret;
    }



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
     * Get gdc attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure
     */
    bool Get_grid_color_gdc(uint32_t *number);	

    /**
     * Set gdc attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_grid_color_gdc(uint32_t number);			
	
    /**
     * Get gdw attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure
     */
    bool Get_grid_width_gdw(uint32_t *number);	

    /**
     * Set gdw attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_grid_width_gdw(uint32_t number);			
	
    /**
     * Get gdh attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure
     */
    bool Get_grid_height_gdh(uint32_t *number);

    /**
     * Set gdh attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_grid_height_gdh(uint32_t number);			
	
    /**
     * Get pco<x> attribute of component
     *
     * @param ch - channel of waveform(0-3). 
     * @param number - buffer storing data retur
     * @return true if success, false for failure
     */
    bool Get_channel_color(uint8_t ch, uint32_t *number);	

    /**
     * Set pco<x> attribute of component
     *
     * @param ch - channel of waveform(0-3). 
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_channel_color(uint8_t ch, uint32_t number);

    /**
     *  Clear waveform component
     * @param ch - channel of waveform(0-3). 255 Clear all data
     * @return true if success, false for failure
     * */
    bool Clear(uint8_t ch);

private:

    /**
     *  Scale value to form
     * 
     * @tparam T - value type
     * @param value - value to convert
     * @return scaled value
     * */
    template<typename T>
    inline uint8_t ScaleToForm(T value)
    {
        if(value > m_maxVal){value=m_maxVal;}
        else if(value < m_minVal){value=m_minVal;}
        value-= m_minVal;
        value *= m_scale;
        uint8_t v = abs((int16_t)value);
        return v;
    }
    float m_minVal;
    float m_maxVal;
    float m_scale;
    uint8_t m_hight;
};

/**
 * @}
 */
