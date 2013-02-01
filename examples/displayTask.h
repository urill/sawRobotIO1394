/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _displayTask_h
#define _displayTask_h

#include <ostream>
#include <cisstMultiTask/mtsTaskContinuous.h>

#include <cisstVector/vctDynamicVectorTypes.h>

class displayTask: public mtsTaskContinuous {
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

protected:
    std::stringstream debugStream;
    enum { DEBUG_START_LINE = 15 };
    unsigned int last_debug_line;
    bool power_on;

    struct RobotStruct {
        mtsFunctionRead GetNumberOfJoints;
        mtsFunctionRead IsValid;
        mtsFunctionVoid EnablePower;
        mtsFunctionVoid DisablePower;
        mtsFunctionRead GetPositionRaw;
        mtsFunctionRead GetVelocityRaw;
        mtsFunctionRead GetAnalogInputRaw;
        mtsFunctionRead GetMotorCurrentRaw;
        mtsFunctionRead GetAmpEnable;
        mtsFunctionRead GetAmpStatus;
        mtsFunctionWrite SetMotorCurrentRaw;
    } Robot;

    vctLongVec posRaw;
    vctLongVec velRaw;
    vctLongVec analogInRaw;
    vctLongVec motorFeedbackCurrentRaw;
    vctLongVec motorControlCurrentRaw;
    vctBoolVec ampEnable;
    vctBoolVec ampStatus;

public:
    displayTask(const std::string & taskName);
    ~displayTask();
    void Configure(const std::string & CMN_UNUSED(filename) = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    std::stringstream &GetOutputStream(void) { return debugStream; }
};

CMN_DECLARE_SERVICES_INSTANTIATION(displayTask)

#endif // _displayTask_h
