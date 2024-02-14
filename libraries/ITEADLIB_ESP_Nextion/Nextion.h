/**
 * @file Nextion.h
 *
 * The header file including all other header files provided by this library. 
 *
 * Every example sketch should include this file. 
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/12
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
 */

#pragma once

#include "Arduino.h"
#include "NexConfig.h"
#include "NexHardwareInterface.h"
#include "NextionIf.h"
#include "NexTouch.h"
#include "NexHardware.h"

#include "NexButton.h"
#include "NexCheckbox.h"
#include "NexCrop.h"
#include "NexDualStateButton.h"
#include "NexGauge.h"
#include "NexGpio.h"
#include "NexHotspot.h"
#include "NexNumber.h"
#include "NexPage.h"
#include "NexPicture.h"
#include "NexProgressBar.h"
#include "NexRadio.h"
#include "NexRtc.h"
#include "NexScreen.h"
#include "NexScrolltext.h"
#include "NexSlider.h"
#include "NexText.h"
#include "NexTimer.h"
#ifdef NEX_ENABLE_TFT_UPLOAD
#include "NexUpload.h"
#endif
#include "NexVariable.h"
#include "NexWaveform.h"
