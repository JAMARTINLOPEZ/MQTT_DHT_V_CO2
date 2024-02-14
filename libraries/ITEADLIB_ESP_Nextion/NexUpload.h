/**
 * @file NexUpload.h
 *
 * The definition of class NexUpload. 
 *
 * @author Chen Zengpeng (email:<zengpeng.chen@itead.cc>)
 * @date 2016/3/29
 * @author Jyrki Berg 2/24/2019 (https://github.com/jyberg)
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
 * 
 */

#pragma once
#ifdef NEX_ENABLE_TFT_UPLOAD

#ifdef ESP8266
//#include <SD.h>
#include <FS.h>
#else
#include <SdFat.h>
#endif
#include <WString.h>
#include "NextionIf.h"


class Nextion;

/**
 * @addtogroup CoreAPI 
 * @{ 
 */

/**
 *
 * Provides the API for nextion to download the ftf file.
 * 
 * https://nextion.tech/2017/12/08/nextion-hmi-upload-protocol-v1-1/
 * 
 */
class NexUpload:public NextionIf
{
    NexUpload()=delete;

public: /* methods */

    /**
     * Constructor. 
     * 
     * @param nextion - nextion iterface
     */
    NexUpload(Nextion *nextion);
    
    /**
     * destructor. 
     * 
     */
    ~NexUpload();
    
    /**
     * start download.
     * 
     * @param tftFile - tft file.
     *
     * @return true if success, false for failure. 
     */
    bool upload(File  &tftFile);
private: /* methods */
    
    /**
     * Re-initialize connection
     * 
     * Ensure that Nextion is in passive mode:
     * https://nextion.tech/2017/12/08/nextion-hmi-upload-protocol-v1-1/
     * 
     */
    void Reconnect();

    /**
     * Initialize upload.
     * 
     * @param tftFile - tft file.
     *
     * @return true if success, false for failure. 
     */
    bool InitUpload(File  &tftFile);
    
    /**
     * start dowload tft file to nextion. 
     * 
     * @param tftFile - tft file.
     * 
     * @return true if success, false for failure.  
     */
    bool _downloadTftFile(File  &tftFile);
};
/**
 * @}
 */
#endif
