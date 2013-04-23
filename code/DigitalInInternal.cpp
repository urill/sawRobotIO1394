/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 $Id$

 Author(s):  Zihan Chen, Peter Kazanzides
 Created on: 2012-07-31

 (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <iostream>

#include <cisstCommonXML.h>
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstParameterTypes/prmEventButton.h>

#include "DigitalInInternal.h"
#include "AmpIO.h"

mtsRobotIO1394::DigitalInInternal::DigitalInInternal(const cmnGenericObject & owner, const std::string &name,
                                                     AmpIO *board, const int bitID) :
    OwnerServices(owner.Services()),
    inputName(name), board(board), bitID(bitID),
    pressDir(false),triggerPress(false), triggerRelease(false),
    curValue(false),previousValue(false)
{
}

mtsRobotIO1394::DigitalInInternal::~DigitalInInternal()
{
}

void mtsRobotIO1394::DigitalInInternal::Configure(cmnXMLPath &xmlConfigFile, int inputNumber){
    // This configuration will go by default method.
    // Will iterate and initialize all digital inputs

    std::string context = "Config";
    std::string tmpTrigger = "";
    int tmpPressed = -1;

    char path[64];
    //Set bitID of input.
    sprintf(path,"DigitalIn[%i]/@BitID",inputNumber);
    xmlConfigFile.GetXMLValue(context.c_str(),path, bitID);
    SetupMasks(bitID,mask);

    //Set pressed status. (Not sure if working)
    sprintf(path,"DigitalIn[%i]/@Pressed",inputNumber);
    xmlConfigFile.GetXMLValue(context.c_str(),path, tmpPressed);
    if(tmpPressed == 0) {
        pressDir = false;
    }
    else if(tmpPressed == 1) {
        pressDir = true;
    }
    else {
        CMN_LOG_RUN_ERROR<<"Setting for "<<path<<" failed."<<std::endl;
        pressDir = false;
    }

    sprintf(path,"DigitalIn[%i]/@Trigger",inputNumber);
    xmlConfigFile.GetXMLValue(context.c_str(),path, tmpTrigger);

    if(tmpTrigger == "all"){
        triggerPress = true;
        triggerRelease = true;
    }
    else if(tmpTrigger == "press"){
        triggerPress = true;
        triggerRelease = false;
    }
    else if(tmpTrigger == "release") {
        triggerPress = false;
        triggerRelease = true;
    }
    else if(tmpTrigger == "none") {
        triggerPress = false;
        triggerRelease = false;
    }
    else {
        //Should not come here during init.
        CMN_LOG_INIT_ERROR<<"Unacceptable Trigger argument: "<<tmpTrigger<<"."<<std::endl;
        CMN_LOG_INIT_ERROR<<"Trigger argument should be one of these: all,press,release,none."<<std::endl;
        triggerPress = false;
        triggerRelease = false;
    }
}

void mtsRobotIO1394::DigitalInInternal::GetData(void){
    //This function will read from the Firewire and use the mask to update boolean state value.
    //Assumed this function would be called 'AFTER' configuration of board and mask is done.
    previousValue = curValue;           // Save old value for nex iteration comparison.
    AmpIO_UInt32 tmpDigitalRead;        // Unsigned int digital read from Firewire.
    tmpDigitalRead = board->GetDigitalInput();          // Read from Firewire
    AmpIO_UInt32 maskResult = tmpDigitalRead & mask;    // Mask the read value from Firewire. Returns corresponding bit.
    bool boolResult = (bool)maskResult;

    //Account for press direction assignment.
    if(pressDir) {
        curValue = boolResult;
    }
    else if(!pressDir) {
        curValue = !boolResult;
    }
    else {
        //This should NEVER happen.
        CMN_LOG_RUN_ERROR<<"Current boolean state assignment fail from "<< bitID <<" of "<< inputName << std::endl;
    }

    prmEventButton buttonPayload;

    if (curValue != previousValue) {
        //There's a change is state. Determine payload and send.
        if(boolResult)  // Rising edge
        {
            if(pressDir) // Active high
            {
                if(triggerPress) // Rising edge active high event trigger enabled
                {
                    buttonPayload = prmEventButton::PRESSED;
                    Button(buttonPayload);
                }
            }
            else if(!pressDir)  // Active low
            {
                if(triggerRelease)  // Rising edge active low event trigger enabled
                {
                    buttonPayload = prmEventButton::RELEASED;
                    Button(buttonPayload);
                }
            }
        }
        else if(!boolResult)    // Falling edge
        {
            if(pressDir)    // Active high
            {
                if(triggerRelease)  // Falling edge active high event trigger enabled
                {
                    buttonPayload = prmEventButton::RELEASED;
                    Button(buttonPayload);
                }
            }
            else if(!pressDir)  // Active low
            {
                if(triggerPress) // Falling edge active low event trigger enabled
                {
                    buttonPayload = prmEventButton::PRESSED;
                    Button(buttonPayload);
                }
            }
        }
    }
}

void mtsRobotIO1394::DigitalInInternal::GetName(std::string &placeHolder) const
{
    placeHolder = inputName;
}

void mtsRobotIO1394::DigitalInInternal::SetupMasks(int bitNumber, AmpIO_UInt32 &maskResult) {
    //This function will setup the mask before reading starts.
    //  maskResult = 0x00000000;
    switch(bitNumber){
    case 0:
        maskResult = 1;     // 0x001
	break;
    case 1:
        maskResult = 2;     // 0x002
	break;
    case 2:
        maskResult = 4;     // 0x004
	break;
    case 3:
        maskResult = 8;     // 0x008
	break;
    case 4:
        maskResult = 16;    // 0x010
	break;
    case 5:
        maskResult = 32;    // 0x020
	break;
    case 6:
        maskResult = 64;    // 0x040
	break;
    case 7:
        maskResult = 128;   // 0x080
	break;
    case 8:
        maskResult = 256;   // 0x100
	break;
    case 9:
        maskResult = 512;   // 0x200
	break;
    case 10:
        maskResult = 1024;  // 0x400
	break;
    case 11:
        maskResult = 2048;  // 0x800
    break;
    default:
        CMN_LOG_INIT_ERROR<<"Unreasonable bitNumber entered for masking with "<< bitNumber<<" requested."<<std::endl;
        maskResult = 0;
	break;
    }
}

void mtsRobotIO1394::DigitalInInternal::SetupStateTable(mtsStateTable &stateTable) {
    stateTable.AddData(curValue, inputName + "curValue");
}

void mtsRobotIO1394::DigitalInInternal::SetupProvidedInterface(mtsInterfaceProvided *prov, mtsStateTable &stateTable) {
    prov->AddCommandReadState(stateTable,this->curValue,"GetButton");
    prov->AddEventWrite(this->Button,"Button",prmEventButton());
}
