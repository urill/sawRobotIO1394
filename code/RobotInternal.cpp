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
mtsRobotIO1394::RobotInternal::JointInfo::JointInfo() : board(0), axisid(-1)
{}

mtsRobotIO1394::RobotInternal::JointInfo::JointInfo(AmpIO *bptr, int aid): board(bptr), axisid(aid)
{}

// JointInfo Destructor
mtsRobotIO1394::RobotInternal::JointInfo::~JointInfo()
{}

mtsRobotIO1394::RobotInternal::RobotInternal(const std::string &name, size_t numJoints) :
    robotName(name), JointList(numJoints), valid(false),
    ampStatus(numJoints, false), ampEnable(numJoints, false),
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

void mtsRobotIO1394::RobotInternal::SetJointInfo(int index, AmpIO *board, int axis)
{
    JointList[index] = JointInfo(board, axis);
}

void mtsRobotIO1394::RobotInternal::SetupStateTable(mtsStateTable &stateTable)
{
    stateTable.AddData(valid, robotName + "Valid");
    stateTable.AddData(powerStatus, robotName + "PowerStatus");
    stateTable.AddData(safetyRelay, robotName + "SafetyRelay");
    stateTable.AddData(ampStatus, robotName + "AmpStatus");
    stateTable.AddData(ampEnable, robotName + "AmpEnable");
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
    prov->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetAmpEnable, this, "SetAmpEnable",
                          this->ampEnable);

    prov->AddCommandReadState(stateTable, this->ampEnable, "GetAmpEnable");
    prov->AddCommandReadState(stateTable, this->ampStatus, "GetAmpStatus");
    prov->AddCommandReadState(stateTable, this->powerStatus, "GetPowerStatus");
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

bool mtsRobotIO1394::RobotInternal::CheckIfValid(void)
{
    size_t i;
    for (i = 0; i < JointList.size(); i++) {
        JointInfo &jt = JointList[i];
        if (!jt.board || (jt.axisid < 0)) break;  // should not happen
        if (!jt.board->ValidRead()) break;
    }
    valid = (i == JointList.size());
    return valid;
}

void mtsRobotIO1394::RobotInternal::GetData(void)
{
    powerStatus = true;
    safetyRelay = true;
    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;
        encPosRaw[index] = board->GetEncoderPosition(axis);
        encVelRaw[index] = board->GetEncoderVelocity(axis);
        analogInRaw[index] = board->GetAnalogInput(axis);
        motorFeedbackCurrentRaw[index] = board->GetMotorCurrent(axis);
        ampEnable[index] = board->GetAmpEnable(axis);
        ampStatus[index] = board->GetAmpStatus(axis);
        powerStatus &= board->GetPowerStatus();
        safetyRelay &= board->GetSafetyRelayStatus();
    }
}

void mtsRobotIO1394::RobotInternal::ConvertRawToSI(void)
{
    EncoderToDegree(encPosRaw, encPos);
    EncoderToDegPerSec(encVelRaw, encVel);
    ADCToVolts(analogInRaw, analogIn);
    ADCToMotorCurrent(motorFeedbackCurrentRaw, motorFeedbackCurrent);
}

//************************ PROTECTED METHODS ******************************

void mtsRobotIO1394::RobotInternal::GetNumberOfJoints(int &num) const
{
    num = JointList.size();
}

void mtsRobotIO1394::RobotInternal::EnablePower(void)
{
    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;
        // Make sure all boards are enabled
        board->SetPowerEnable(true);
        // For now, also enable all amplifiers
        board->SetAmpEnable(axis, true);
    }
}

void mtsRobotIO1394::RobotInternal::DisablePower(void)
{
    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;
        // Make sure all boards are disabled
        board->SetPowerEnable(false);
        // For now, also disable all amplifiers
        board->SetAmpEnable(axis, false);
    }
}

void mtsRobotIO1394::RobotInternal::EnableSafetyRelay(void)
{
    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;
        board->SetSafetyRelay(true);
    }
}

void mtsRobotIO1394::RobotInternal::DisableSafetyRelay(void)
{
    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;
        board->SetSafetyRelay(false);
    }
}

void mtsRobotIO1394::RobotInternal::SetAmpEnable(const vctBoolVec &ampControl)
{
    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;
        board->SetAmpEnable(axis, ampControl[index]);
    }
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
    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;
        board->SetMotorCurrent(axis, motorControlCurrentRaw[index]);
    }
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
    SetMotorCurrentRaw(motorControlCurrentRaw);
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
