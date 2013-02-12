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
#include <cisstCommon/cmnLogger.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>

#include "RobotInternal.h"
#include "AmpIO.h"


// ZC: MOVE to configuration file
const unsigned int      ENC_CPT  =      4000;    // OK  1000 x 4 quadrature
const unsigned long ENC_VEL_MAX  =  0x00FFFF;    // TEMP need check // maximum value of encoder pulse period
const double        ENC_VEL_CLK  = 1000000.0;    // TMEP need check clock (Hz) used to measure encoder pulse
const double        ENC_ACC_CLK  =      12.0;    // TEMP need check
const unsigned long MIDRANGE_ADC = 0x0008000;    // 16 bits ADC mid range value
const unsigned long ENC_OFFSET   = 0x007FFFFF;   // Encoder offset


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
    motorControlCurrentRaw(numJoints), motorControlCurrent(numJoints),
    encSetPosRaw(numJoints), encSetPos(numJoints)
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
    prov->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetEncoderPositionRaw, this, "SetEncoderPositionRaw",
                          encSetPosRaw);
    prov->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetEncoderPosition, this, "SetEncoderPosition",
                          encSetPos);

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
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::PotVoltsToDegree, this,
                                  "PotVoltsToDegree", analogIn, analogIn); //TODO: CHECK this
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
        encPosRaw[index] = board->GetEncoderPosition(axis) - ENC_OFFSET;
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



void mtsRobotIO1394::RobotInternal::SetEncoderPositionRaw(const vctLongVec &epos)
{
    if (epos.size() != encSetPosRaw.size()) {
        CMN_LOG_RUN_ERROR << robotName << "::SetEncoderPositionRaw: size mismatch ("
                          << epos.size() << ", " << encSetPosRaw.size() << ")" << std::endl;
        return;
    }
    encSetPosRaw = epos;
    for (size_t index = 0; index < JointList.size(); index++){
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;
        board->WriteEncoderPreload(axis, encSetPosRaw[index] + ENC_OFFSET);
    }
}

void mtsRobotIO1394::RobotInternal::SetEncoderPosition(const vctDoubleVec &epos)
{
    if (epos.size() != encSetPos.size()) {
        CMN_LOG_RUN_ERROR << robotName << "::SetEncoderPosition: size mismatch ("
                          << epos.size() << ", " << encSetPos.size() << ")" << std::endl;
        return;
    }
    encSetPos = epos;
    DegreeToEncoder(encSetPos, encSetPosRaw);
    SetEncoderPositionRaw(encSetPosRaw);
}


// Unit Conversions (TBD)
void mtsRobotIO1394::RobotInternal::EncoderToDegree(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);

    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;

        int countsperturn = JointList[index].countsperturn;
        double pitch = JointList[index].pitch;
        double gearratio = JointList[index].gearratio;

        // divide by CPT get get number of turns
        // multiply by 360 degs per turn
        // multiply by Pitch (1 for revolute, n mm/deg for prismatic joint)
        // divide by gearratio to get joint deg
        toData[index] = 360.0 * fromData[index] / countsperturn
                * pitch / gearratio;
    }
}

// used in SetPosition
void mtsRobotIO1394::RobotInternal::DegreeToEncoder(const vctDoubleVec &fromData, vctLongVec &toData) const
{
    toData.SetAll(0L);

    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;

        int countsperturn = JointList[index].countsperturn;
        double pitch = JointList[index].pitch;
        double gearratio = JointList[index].gearratio;

        // multiply by gearratio to get motor degree
        // divide by pitch (revolute:1 prismatic:mm/deg)
        // divide by 360 to get num of turn
        // multiply by CPT to get number of counts
        toData[index] = (fromData[index]) * gearratio / pitch
                * countsperturn / 360.0;
    }
}

void mtsRobotIO1394::RobotInternal::EncoderToDegPerSec(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);

    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;

        int countsperturn = JointList[index].countsperturn;

        // negate for sign to match that of motor command voltage
        // subtract half of max to get direction/sign of motion
        // divide clocks/s by #clocks to get counts per second
        // divide by CPT to get number of turns per second
        // mutiply by 360 degrees per turn for deg/s
        toData[index] = (-360.0 * ENC_VEL_CLK /
                                   ((double)fromData[index]- ENC_VEL_MAX/2) / (countsperturn/4.0));
    }
}

void mtsRobotIO1394::RobotInternal::MotorCurrentToDAC(const vctDoubleVec &fromData, vctLongVec &toData) const
{
    toData.SetAll(0L);

    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;

        int currenttocnts = JointList[index].currenttocnts;

        // conversion
        // DAC 16 bits 0x0000 - 0xffff  mid 0x8000
        long dacCounts;
        dacCounts = static_cast<long>(currenttocnts * fromData[index]) + 0x8000;
        if (dacCounts > 0xffff) {dacCounts = 0xffff;}
        if (dacCounts < 0x0000) {dacCounts = 0x0000;}
        toData[index] = dacCounts;
    }
}

void mtsRobotIO1394::RobotInternal::ADCToVolts(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);

    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;

        double cntstoanalogvolt = JointList[index].cntstoanalogvolt;

        // conversion
        toData[index] = cntstoanalogvolt * fromData[index];
    }
}

void mtsRobotIO1394::RobotInternal::ADCToMotorCurrent(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);

    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;

        double cntstocurrent = JointList[index].cntstocurrent;

        // conversion
        toData[index] = cntstocurrent * (fromData[index]-MIDRANGE_ADC);
    }
}


void mtsRobotIO1394::RobotInternal::PotVoltsToDegree(const vctDoubleVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);

    for (size_t index = 0; index < JointList.size(); index++) {
        AmpIO *board = JointList[index].board;
        int axis = JointList[index].axisid;
        if (!board || (axis < 0)) continue;

        double slope = JointList[index].slope;
        double intercept = JointList[index].intercept;

        // conversion
        toData[index] = fromData[index] * slope + intercept;
    }
}



//ZC: parse XML file for jointinfo
void mtsRobotIO1394::RobotInternal::Configure(const std::string &filename)
{
    // TODO: replace Robot with robot name
    if (filename == "") {
        CMN_LOG_INIT_ERROR << "Configure: could not configure Robot" << std::endl;
        return;
    }

    // log configure file path
    CMN_LOG_INIT_VERBOSE << "Configuring Robot with \"" << filename << "\"" << std::endl;

    // set cmnXMLPath
    cmnXMLPath xmlConfig;
    xmlConfig.SetInputSource(filename);

    char path[64];
    int numOfBoards;
    int numOfJoints;
    std::string context("/config/robot");
    xmlConfig.GetXMLValue(context.c_str(), "/@name", robotName);
    xmlConfig.GetXMLValue(context.c_str(), "/@numofboards", numOfBoards);
    xmlConfig.GetXMLValue(context.c_str(), "/@numofjoints", numOfJoints);

    std::cout << "name: " << robotName << std::endl;
    std::cout << "numofboards: " << numOfBoards << std::endl;
    std::cout << "numofjoints: " << numOfBoards << std::endl;

    // read board info
    for (int i = 0; i < numOfBoards; i++){
        int bid;
        std::string type;
        sprintf(path, "board[%d]/@id", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, bid);
        sprintf(path, "board[%d]/@id", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, type);
    }

    // read in the conversion factors
    for (int i = 0; i < numOfJoints; i++) {

        // joint
        int index, boardid, axisid;
        sprintf(path, "joints/joint[%d]/@index", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, index);
        sprintf(path, "joints/joint[%d]/@boardid", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, boardid);
        sprintf(path, "joints/joint[%d]/@axisid", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, axisid);

        // encoder
        sprintf(path, "joints/joint[%d]/encoder/@countsperturn", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].countsperturn);
        sprintf(path, "joints/joint[%d]/encoder/@gearratio", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].gearratio);
        sprintf(path, "joints/joint[%d]/encoder/@pitch", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].pitch);
        sprintf(path, "joints/joint[%d]/encoder/@offsetdeg", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].offsetdeg);

        // motor
        sprintf(path, "joints/joint[%d]/motor/@ktorque", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].torquecurrent);

        // pot
        sprintf(path, "joints/joint[%d]/pot/@maxvol", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].maxpotvolt);
        sprintf(path, "joints/joint[%d]/pot/@slope", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].slope);
        sprintf(path, "joints/joint[%d]/pot/@intercept", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].intercept);

        // current
        sprintf(path, "joints/joint[%d]/current/@maxcur", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].maxcurrent);

        // adc
        sprintf(path, "joints/joint[%d]/adc/@cntstocurrent", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].cntstocurrent);
        sprintf(path, "joints/joint[%d]/adc/@cntstoanalogvolt", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].cntstoanalogvolt);

        // dac
        sprintf(path, "joints/joint[%d]/dac/@currenttocnts", i+1);
        xmlConfig.GetXMLValue(context.c_str(), path, JointList[index].currenttocnts);
    }
    CMN_LOG_INIT_VERBOSE << "Configured RobotInternal" << std::endl;
}
















