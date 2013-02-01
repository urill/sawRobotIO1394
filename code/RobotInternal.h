/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$
  
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10
  
  (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef __RobotInternal_H__
#define __RobotInternal_H__

#include <string>
#include <vector>

#include <cisstVector/vctDynamicVectorTypes.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>

class mtsInterfaceProvided;
class mtsStateTable;

class mtsRobotIO1394::RobotInternal {
public:
    //Nested class stores Joint-Axis info
    class JointInfo {
      public:
        int boardid;
        int axisid;
      public:
        JointInfo();
        JointInfo(int bid, int aid);
        ~JointInfo();
    };
    std::string robotName;              // Robot name (from config file)
    std::vector<JointInfo> JointList;   // Joint information

    // State data
    bool           valid;
    unsigned short safetyRelay;
    bool           safetyRelayControl;
    vctBoolVec     ampStatus;
    vctBoolVec     ampEnable;
    vctLongVec     encPosRaw;
    vctDoubleVec   encPos;
    vctLongVec     encVelRaw;
    vctDoubleVec   encVel;
    vctLongVec     analogInRaw;
    vctDoubleVec   analogIn;
    vctLongVec     motorFeedbackCurrentRaw;
    vctDoubleVec   motorFeedbackCurrent;
    vctLongVec     motorControlCurrentRaw;
    vctDoubleVec   motorControlCurrent;

    // Methods for provided interface
    void GetNumberOfJoints(int &num) const;
    void EnablePower(void);
    void DisablePower(void);
    void EnableSafetyRelay(void);
    void DisableSafetyRelay(void);
    void SetMotorCurrentRaw(const vctLongVec &mcur);
    void SetMotorCurrent(const vctDoubleVec &mcur);

    // Unit conversion Raw -- SI (partial implementation)
    void EncoderToDegree(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void DegreeToEncoder(const vctDoubleVec &fromData, vctLongVec &toData) const;
    void EncoderToDegPerSec(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void MotorCurrentToDAC(const vctDoubleVec &fromData, vctLongVec &toData) const;
    void ADCToVolts(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void ADCToMotorCurrent(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void ConvertRawToSI(void);

    RobotInternal(const std::string &name, size_t numJoints);
    ~RobotInternal();

    void SetupStateTable(mtsStateTable &stateTable);
    void SetupProvidedInterface(mtsInterfaceProvided *prov, mtsStateTable &stateTable);

    void SetValid(bool flag) { valid = flag; }
    bool IsValid(void) const { return valid; }
};

#endif  //__RobotInternal_H__
