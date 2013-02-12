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
class AmpIO;

class mtsRobotIO1394::RobotInternal {
protected:
    //Nested class stores Joint-Axis info
    class JointInfo {
      public:
        AmpIO *board;
        int axisid;

        // ZC: temp, may change based on final xml file
        // encoder
        int countsperturn;
        double gearratio;
        double pitch;
        double offsetdeg;
        // motor
        double torquecurrent;
        // pot
        double maxpotvolt;
        double slope;
        double intercept;
        // current
        double maxcurrent;
        // adc
        double cntstocurrent;
        double cntstoanalogvolt;
        // dac
        double currenttocnts;

      public:
        JointInfo();
        JointInfo(AmpIO *bptr, int aid);
        ~JointInfo();
    };
    std::string robotName;              // Robot name (from config file)
    std::vector<JointInfo> JointList;   // Joint information

    // State data
    bool           valid;
    bool           powerStatus;
    unsigned short safetyRelay;
    vctBoolVec     ampStatus;           // Amplifier actual status (ON or FAULT)
    vctBoolVec     ampEnable;           // Current amplifier enable state (read from boards)
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
    vctLongVec     encSetPosRaw;
    vctDoubleVec   encSetPos;

    // Methods for provided interface
    void GetNumberOfJoints(int &num) const;
    void EnablePower(void);
    void DisablePower(void);
    void EnableSafetyRelay(void);
    void DisableSafetyRelay(void);
    void SetAmpEnable(const vctBoolVec &ampControl);
    void SetMotorCurrentRaw(const vctLongVec &mcur);
    void SetMotorCurrent(const vctDoubleVec &mcur);
    void SetEncoderPositionRaw(const vctLongVec &epos);
    void SetEncoderPosition(const vctDoubleVec &epos);
//    void SetEncoderFromPot(void);  // TODO: Implement 

    // Unit conversion Raw -- SI (partial implementation)
    void EncoderToDegree(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void DegreeToEncoder(const vctDoubleVec &fromData, vctLongVec &toData) const;
    void EncoderToDegPerSec(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void MotorCurrentToDAC(const vctDoubleVec &fromData, vctLongVec &toData) const;
    void ADCToVolts(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void ADCToMotorCurrent(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void PotVoltsToDegree(const vctDoubleVec &fromData, vctDoubleVec &toData) const;

public:

    RobotInternal(const std::string &name, size_t numJoints);
    ~RobotInternal();

    // ZC: parse XML file here
    void Configure(const std::string &filename);

    void SetJointInfo(int index, AmpIO *board, int axis);

    void SetupStateTable(mtsStateTable &stateTable);
    void SetupProvidedInterface(mtsInterfaceProvided *prov, mtsStateTable &stateTable);

    bool CheckIfValid(void);
    bool IsValid(void) const { return valid; }

    void GetData(void);
    void ConvertRawToSI(void);
};

#endif  //__RobotInternal_H__
