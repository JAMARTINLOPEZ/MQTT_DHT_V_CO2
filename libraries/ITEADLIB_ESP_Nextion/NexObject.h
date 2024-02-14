/**
 * @file NexObject.h
 *
 * The definition of class NexObject. 
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

//#include <Arduino.h>
//#include "Nextion.h"
//#include "NexConfig.h"
#include "NextionIf.h"

class Nextion;

/**
 * @addtogroup CoreAPI 
 * @{ 
 */

/**
 * Root class of all Nextion components. 
 *
 * Provides the essential attributes of a Nextion component and the methods accessing
 * them. At least, Page ID(pid), Component ID(pid) and an unique name are needed for
 * creating a component in Nexiton library. 
 */
class NexObject : public NextionIf
{
    NexObject()=delete;

public: /* methods */

    /**
     * Constructor. 
     *
     * @param nextion - nextion interface
     * @param pid - page id. 
     * @param cid - component id.    
     * @param name - pointer to an unique name in range of all components.
     * @param page - pointer to global page information (can be nullptr in case local object)
     */
    NexObject(Nextion *nextion, uint8_t pid, uint8_t cid, const char* name, const NexObject* page);

    /**
     * Get object width
     * 
     * @param width - buffer storing data return
     * @return true if success, false for failure
     */
    bool GetObjectWidth( uint32_t &width); 

    /**
     * Get object height
     * 
     * @param height - buffer storing data return
     * @return true if success, false for failure
     */
    bool GetObjectHeight( uint32_t &height); 

    /**
     * Print current object'address, page id, component id and name. 
     *
     * @warning this method does nothing, unless debug message enabled. 
     */
    void printObjInfo(void);

    /**
     * Hide or Show componen on current page
     *
     * @param visible - true Show component, false Hide component 
     * @return true if success, false for failure
     */
    bool setVisible(bool visible);
    
    /**
     * Refresh componen on current page
     *
     * @return true if success, false for failure
     */
    bool refresh();

protected: /* methods */

    /*
     * Get page id.
     *
     * @return the id of page.  
     */
    uint8_t getObjPid(void);    

    /*
     * Get component id.
     *
     * @return the id of component.  
     */
    uint8_t getObjCid(void);

    /*
     * Get component name.
     *
     * @return the name of component. 
     */
    const char *getObjName(void) const;    

    /*
     * Get component page name.
     *
     * @return the name of component page, nullptr if not defined (local). 
     */    
    const char* getObjPageName(void);

    /*
    * Get component global name
    * 
    * @param gName - object page name
    */
    void getObjGlobalPageName(String &gName);

private: /* data */ 
    const uint8_t _pid; /* Page ID */
    const uint8_t _cid; /* Component ID */
    const char* _name; /* An unique name */
    const NexObject* _page; /* page information for global objects nullptr for local */
};
/**
 * @}
 */
