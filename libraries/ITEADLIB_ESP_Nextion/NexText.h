/**
 * @file NexText.h
 *
 * The definition of class NexText. 
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
 * NexText component.
 */
class NexText: public NexTouch
{
    NexText()=delete;
    
public: /* methods */

    /**
     * @copydoc NexObject::NexObject(Nextion*, uint8_t, uint8_t, const char*, const NexObject*)
     */
    NexText(Nextion *nextion, uint8_t pid, uint8_t cid, const char *name, const NexObject* page=nullptr);
    
    /*
    * Get text attribute of component. 
    * 
    * @param str - String storing text returned. 
    *
    * @retval true - success. 
    * @retval false - failed.
    *
    */
    bool getText(String &str);

    /**
     * Get text attribute of component.
     *
     * @param buffer - buffer storing text returned. 
     * @param len - in buffer len / out saved string len excluding null char.  
     * 
     * @retval true - success. 
     * @retval false - failed.
     */
    bool getText(char *buffer, uint16_t &len);
    
    /**
     * Set text attribute of component.
     *
     * @param buffer - text buffer terminated with '\0'. 
     * @return true if success, false for failure. 
     */
    bool setText(const char *buffer);    
	
    /**
     * Append text to text attribute of component.
     *
     * @param buffer - text buffer terminated with '\0'. 
     * @return true if success, false for failure. 
     */
    bool appendText(const char *buffer);

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
     * Get xcen attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure 
     */
    bool Get_place_xcen(uint32_t *number);	

    /**
     * Set xcen attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_place_xcen(uint32_t number);			
	
    /**
     * Get ycen attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure 
     */
    bool Get_place_ycen(uint32_t *number);	

    /**
     * Set ycen attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_place_ycen(uint32_t number);			
	
    /**
     * Get font attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure 
     */
    bool getFont(uint32_t *number);		
	
    /**
     * Set font attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool setFont(uint32_t number);			
	
    /**
     * Get picc attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure 
     */
    bool Get_background_crop_picc(uint32_t *number);	

    /**
     * Set picc attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_background_crop_picc(uint32_t number);			
	
    /**
     * Get pic attribute of component
     *
     * @param number - buffer storing data retur
     * @return true if success, false for failure 
     */
    bool Get_background_image_pic(uint32_t *number);	

    /**
     * Set pic attribute of component
     *
     * @param number - To set up the data
     * @return true if success, false for failure
     */
    bool Set_background_image_pic(uint32_t number);	
    
};

/**
 * @}
 */
