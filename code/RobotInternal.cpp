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

#include <cisstCommon/cmnLogger.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>

#include "RobotInternal.h"
#include "AmpIO.h"

// JointInfo Constructor
mtsRobotIO1394::RobotInternal::JointInfo::JointInfo() : boardid(-1), axisid(-1)
{}

mtsRobotIO1394::RobotInternal::JointInfo::JointInfo(int bid, int aid): boardid(bid), axisid(aid)
{}

// JointInfo Destructor
mtsRobotIO1394::RobotInternal::JointInfo::~JointInfo()
{}

mtsRobotIO1394::RobotInternal::RobotInternal(const std::string &name, size_t numJoints) :
    robotName(name), JointList(numJoints), valid(false),
    ampStatus(numJoints, false), ampEnable(numJoints, false), ampControl(numJoints, false),
    encPosRaw(numJoints), encPos(numJoints),
    encVelRaw(numJoints), encVel(numJoints),
    analogInRaw(numJoints), analogIn(numJoints),
    motorFeedbackCurrentRaw(numJoints), motorFeedbackCurrent(numJoints),
    motorControlCurrentRaw(numJoints), motorControlCurrent(numJoints)
{
}

mtsRobotIO1394::RobotInternal::~RobotInternal()
{
}

void mtsRobotIO1394::RobotInternal::SetupStateTable(mtsStateTable &stateTable)
{
    stateTable.AddData(valid, robotName + "Valid");
    stateTable.AddData(safetyRelay, robotName + "SafetyRelay");
    stateTable.AddData(safetyRelayControl, robotName + "SafetyRelayControl");
    stateTable.AddData(ampStatus, robotName + "AmpStatus");
    stateTable.AddData(ampEnable, robotName + "AmpEnable");
    stateTable.AddData(ampControl, robotName + "AmpControl");
    stateTable.AddData(encPosRaw, robotName + "PosRaw");
    stateTable.AddData(encPos, robotName + "Pos");
    stateTable.AddData(encVelRaw, robotName + "VelRaw");
    stateTable.AddData(encVel, robotName + "Vel");
    stateTable.AddData(analogInRaw, robotName + "AnalogInRaw");
    stateTable.AddData(analogIn, robotName + "AnalogIn");
    stateTable.AddData(motorFeedbackCurrentRaw, robotName + "MotorFeedbackCurrentRaw");
    stateTable.AddData(motorFeedbackCurrent, robotName + "MotorFeedbackCurrent");
    stateTable.AddData(motorControlCurrentRaw, robotName + "MotorControlCurrentRaw");
    stateTable.AddData(motorControlCurrent, robotName + "MotorControlCurrent");
}

void mtsRobotIO1394::RobotInternal::SetupProvidedInterface(mtsInterfaceProvided *prov, mtsStateTable &stateTable)
{
    prov->AddCommandRead(&mtsRobotIO1394::RobotInternal::GetNumberOfJoints, this,
                         "GetNumberOfJoints");
    prov->AddCommandReadState(stateTable, this->valid, "IsValid");

    // Enable // Disable
    prov->AddCommandVoid(&mtsRobotIO1394::RobotInternal::EnablePower, this, "EnablePower");
    prov->AddCommandVoid(&mtsRobotIO1394::RobotInternal::DisablePower, this, "DisablePower");
    prov->AddCommandVoid(&mtsRobotIO1394::RobotInternal::EnableSafetyRelay, this, "EnableSafetyRelay");
    prov->AddCommandVoid(&mtsRobotIO1394::RobotInternal::DisableSafetyRelay, this, "DisableSafetyRelay");

    prov->AddCommandReadState(stateTable, this->ampEnable, "GetAmpEnable");
    prov->AddCommandReadState(stateTable, this->ampStatus, "GetAmpStatus");
    prov->AddCommandWriteState(stateTable, this->ampControl, "SetAmpStatus");
    prov->AddCommandReadState(stateTable, this->safetyRelay, "GetSafetyRelay");

    prov->AddCommandReadState(stateTable, this->encPosRaw, "GetPositionRaw");
    prov->AddCommandReadState(stateTable, this->encPos, "GetPosition");

    prov->AddCommandReadState(stateTable, this->encVelRaw, "GetVelocityRaw");
    prov->AddCommandReadState(stateTable, this->encVel, "GetVelocity");

    prov->AddCommandReadState(stateTable, this->analogInRaw, "GetAnalogInputRaw");
    prov->AddCommandReadState(stateTable, this->analogIn, "GetAnalogInput");

    prov->AddCommandReadState(stateTable, this->motorFeedbackCurrentRaw, "GetMotorFeedbackCurrentRaw");
    prov->AddCommandReadState(stateTable, this->motorFeedbackCurrent, "GetMotorFeedbackCurrent");

    prov->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetMotorCurrentRaw, this, "SetMotorCurrentRaw",
                          motorControlCurrentRaw);
    prov->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetMotorCurrent, this, "SetMotorCurrent",
                          motorControlCurrent);

    // unit conversion methods (Qualified Read)
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::EncoderToDegree, this,
                                  "EncoderToDegree", encPosRaw, encPos);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::DegreeToEncoder, this,
                                  "DegreeToEncoder", encPos, encPosRaw);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::EncoderToDegPerSec, this,
                                  "EncoderToDegPerSec", encVelRaw, encVel);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::MotorCurrentToDAC, this,
                                  "MotorCurrentToDAC", motorControlCurrent, motorControlCurrentRaw);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::ADCToVolts, this,
                                  "ADCToVolts",analogInRaw, analogIn);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::ADCToMotorCurrent, this,
                                  "ADCToMotorCurrent",motorFeedbackCurrentRaw, motorFeedbackCurrent);
}

void mtsRobotIO1394::RobotInternal::GetNumberOfJoints(int &num) const
{
    num = JointList.size();
}

void mtsRobotIO1394::RobotInternal::EnablePower(void)
{
    ampControl.SetAll(true);
}

void mtsRobotIO1394::RobotInternal::DisablePower(void)
{
    ampControl.SetAll(false);
}

void mtsRobotIO1394::RobotInternal::EnableSafetyRelay(void)
{
    safetyRelayControl = true;
}

void mtsRobotIO1394::RobotInternal::DisableSafetyRelay(void)
{
    safetyRelayControl = false;
}

void mtsRobotIO1394::RobotInternal::SetMotorCurrentRaw(const vctLongVec &mcur)
{
    if (mcur.size() != motorControlCurrentRaw.size()) {
        CMN_LOG_RUN_ERROR << robotName << "::SetMotorCurrentRaw: size mismatch ("
                          << mcur.size() << ", " << motorControlCurrentRaw.size() << ")" << std::endl;
        return;
    }
    motorControlCurrentRaw = mcur;
    // TODO: unit conversion to motorControlCurrent
}

void mtsRobotIO1394::RobotInternal::SetMotorCurrent(const vctDoubleVec &mcur)
{
    if (mcur.size() != motorControlCurrent.size()) {
        CMN_LOG_RUN_ERROR << robotName << "::SetMotorCurrent: size mismatch ("
                          << mcur.size() << ", " << motorControlCurrent.size() << ")" << std::endl;
        return;
    }
    motorControlCurrent = mcur;
    MotorCurrentToDAC(motorControlCurrent, motorControlCurrentRaw);
}

// Unit Conversions (TBD)
void mtsRobotIO1394::RobotInternal::EncoderToDegree(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
}

void mtsRobotIO1394::RobotInternal::DegreeToEncoder(const vctDoubleVec &fromData, vctLongVec &toData) const
{
    toData.SetAll(0L);
}

void mtsRobotIO1394::RobotInternal::EncoderToDegPerSec(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
}

void mtsRobotIO1394::RobotInternal::MotorCurrentToDAC(const vctDoubleVec &fromData, vctLongVec &toData) const
{
    toData.SetAll(0L);
}

void mtsRobotIO1394::RobotInternal::ADCToVolts(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
}

void mtsRobotIO1394::RobotInternal::ADCToMotorCurrent(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
}

void mtsRobotIO1394::RobotInternal::GetData(size_t index, const AmpIO *board, int axis)
{
    // Assumes that index and axis were already verified to be in range
    encPosRaw[index] = board->GetEncoderPosition(axis);
    encVelRaw[index] = board->GetEncoderVelocity(axis);
    analogInRaw[index] = board->GetAnalogInput(axis);
    motorFeedbackCurrentRaw[index] = board->GetMotorCurrent(axis);
    ampEnable[index] = board->GetAmpEnable(axis);
    ampStatus[index] = board->GetAmpStatus(axis);
}

void mtsRobotIO1394::RobotInternal::ConvertRawToSI(void)
{
    EncoderToDegree(encPosRaw, encPos);
    EncoderToDegPerSec(encVelRaw, encVel);
    ADCToVolts(analogInRaw, analogIn);
    ADCToMotorCurrent(motorFeedbackCurrentRaw, motorFeedbackCurrent);
}

