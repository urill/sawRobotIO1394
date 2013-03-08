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
    analogInRaw(numJoints), analogInVolts(numJoints), analogInPosSI(numJoints),
    motorFeedbackCurrentRaw(numJoints), motorFeedbackCurrent(numJoints),
    motorControlCurrentRaw(numJoints), motorControlCurrent(numJoints)
{
}

mtsRobotIO1394::RobotInternal::~RobotInternal()
{
}
void mtsRobotIO1394::RobotInternal::Configure(const std::string &filename){
    //This configuration will go by default method. Will assume there is only one robot, and the first one is it.
    //If there are more than one robot per configuration file, then this method should not be used.

    cmnXMLPath xmlConfig;
    xmlConfig.SetInputSource(filename);

    Configure(xmlConfig,1);
}

void mtsRobotIO1394::RobotInternal::Configure (cmnXMLPath  &xmlConfigFile, int robotNumber){
    char path[64];
    std::string context = "Config";

    int  tmpNumOfJoint;
    sprintf(path, "Robot[%i]/@NumOfJoint",robotNumber);
    xmlConfigFile.GetXMLValue(context.c_str(),path,tmpNumOfJoint);

    for(int i=0; i<tmpNumOfJoint; i++)    {
        sprintf(path,"Robot[%i]/Joint[%d]/Drive/AmpsToBits/@Scale",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].drive.AmpsToBitsScale);

        sprintf(path,"Robot[%i]/Joint[%d]/Drive/AmpsToBits/@Offset",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].drive.AmpsToBitsOffset);

        sprintf(path,"Robot[%i]/Joint[%d]/Drive/BitsToFbAmps/@Scale",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].drive.BitsToFbAmpsScale);

        sprintf(path,"Robot[%i]/Joint[%d]/Drive/BitsToFbAmps/@Offset",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].drive.BitsToFbAmpsOffset);

        sprintf(path,"Robot[%i]/Joint[%d]/Drive/NmToAmps/@Scale",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].drive.NmToAmpsScale);

        sprintf(path,"Robot[%i]/Joint[%d]/Drive/MaxCurrent/@Value",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].drive.MaxCurrentValue);

        sprintf(path,"Robot[%i]/Joint[%d]/Encoder/BitsToPosSI/@Scale",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].encoder.BitsToPosSIScale);

        sprintf(path,"Robot[%i]/Joint[%d]/Encoder/BitsToPosSI/@Offset",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].encoder.BitsToPosSIOffset);

        sprintf(path,"Robot[%i]/Joint[%d]/Encoder/BitsToDeltaPosSI/@Scale",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].encoder.BitsToDeltaPosSIScale);

        sprintf(path,"Robot[%i]/Joint[%d]/Encoder/BitsToDeltaPosSI/@Offset",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].encoder.BitsToDeltaPosSIOffset);

        sprintf(path,"Robot[%i]/Joint[%d]/Encoder/BitsToDeltaT/@Scale",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].encoder.BitsToDeltaTScale);

        sprintf(path,"Robot[%i]/Joint[%d]/Encoder/BitsToDeltaT/@Offset",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].encoder.BitsToDeltaTOffset);

        sprintf(path,"Robot[%i]/Joint[%d]/Encoder/CountsPerTurn/@Value",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].encoder.CountsPerTurnValue);

        sprintf(path,"Robot[%i]/Joint[%d]/AnalogIn/BitsToVolts/@Scale",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].analogIn.BitsToVoltsScale);

        sprintf(path,"Robot[%i]/Joint[%d]/AnalogIn/BitsToVolts/@Offset",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].analogIn.BitsToVoltsOffset);

        sprintf(path,"Robot[%i]/Joint[%d]/AnalogIn/VoltsToPosSI/@Scale",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].analogIn.VoltsToPosSIScale);

        sprintf(path,"Robot[%i]/Joint[%d]/AnalogIn/VoltsToPosSI/@Offset",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,JointList[i].analogIn.VoltsToPosSIOffset);
    }
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
    stateTable.AddData(analogInVolts, robotName + "AnalogInVolts");
    stateTable.AddData(analogInPosSI, robotName + "AnalogInPosSI");
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
    prov->AddCommandReadState(stateTable, this->analogInVolts, "GetAnalogInputVolts");
    prov->AddCommandReadState(stateTable, this->analogInPosSI, "GetAnalogInputPosSI");

    prov->AddCommandReadState(stateTable, this->motorFeedbackCurrentRaw, "GetMotorFeedbackCurrentRaw");
    prov->AddCommandReadState(stateTable, this->motorFeedbackCurrent, "GetMotorFeedbackCurrent");

    prov->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetMotorCurrentRaw, this, "SetMotorCurrentRaw",
                          motorControlCurrentRaw);
    prov->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetMotorCurrent, this, "SetMotorCurrent",
                          motorControlCurrent);

    // unit conversion methods (Qualified Read)
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::EncoderRawToSI, this,
                                  "EncoderRawToSI", encPosRaw, encPos);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::EncoderSIToRaw, this,
                                  "EncoderSIToRaw", encPos, encPosRaw);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosSI, this,
                                  "EncoderRawToDeltaPosSI", encVelRaw, encVel);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosT, this,
                                  "EncoderRawToDeltaPosT", encVelRaw, encVel);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::DriveAmpsToBits, this,
                                  "DriveAmpsToFbBits", motorFeedbackCurrent, motorFeedbackCurrentRaw);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::DriveAmpsToNm, this,
                                  "DriveAmpsToNm", motorControlCurrent, motorControlTorque);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::DriveNmToAmps, this,
                                  "DriveNmToAmps", motorControlTorque, motorControlCurrent);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::AnalogInBitsToVolts, this,
                                  "AnalogInBitsToVolts",analogInRaw, analogInVolts);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::AnalogInVoltsToPosSI, this,
                                  "AnalogInVoltsToPosSI",analogInVolts, analogInPosSI);
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
    EncoderRawToSI(encPosRaw, encPos);
    EncoderRawToDeltaPosSI(encVelRaw, encVel);
    AnalogInBitsToVolts(analogInRaw, analogInVolts);
    DriveBitsToFbAmps(motorFeedbackCurrentRaw, motorFeedbackCurrent);
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
    //MotorCurrentToDAC(motorControlCurrent, motorControlCurrentRaw);
    DriveAmpsToBits(motorControlCurrent, motorControlCurrentRaw);
    SetMotorCurrentRaw(motorControlCurrentRaw);
}

// Unit Conversions
void mtsRobotIO1394::RobotInternal::EncoderRawToSI(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index = 0; index<JointList.size();index++){
        double bitsToPosSIScale = JointList[index].encoder.BitsToPosSIScale;
        double bitsToPosSIOffset = JointList[index].encoder.BitsToPosSIOffset;

        toData[index] = (fromData[index] * bitsToPosSIScale) + bitsToPosSIOffset;
    }
}

void mtsRobotIO1394::RobotInternal::EncoderSIToRaw(const vctDoubleVec &fromData, vctLongVec &toData) const
{
    toData.SetAll(0L);
    for (size_t index = 0; index < JointList.size(); index++) {
        double bitsToPosSIScale = JointList[index].encoder.BitsToPosSIScale;
        double bitsToPosSIOffset = JointList[index].encoder.BitsToPosSIOffset;

        toData[index] = (fromData[index]-bitsToPosSIOffset) / bitsToPosSIScale;
    }
}

void mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosSI(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index = 0; index<JointList.size();index++){
        double bitsToDeltaPosSIScale = JointList[index].encoder.BitsToDeltaPosSIScale;
        double bitsToDeltaPosSIOffset = JointList[index].encoder.BitsToDeltaPosSIOffset;

        toData[index] = (fromData[index] * bitsToDeltaPosSIScale) + bitsToDeltaPosSIOffset;
    }
}

void mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosT(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index = 0; index<JointList.size();index++){
        double bitsToDeltaTScale = JointList[index].encoder.BitsToDeltaTScale;
        double bitsToDeltaTOffset = JointList[index].encoder.BitsToDeltaTOffset;

        toData[index]=(fromData[index] * bitsToDeltaTScale) + bitsToDeltaTOffset ;
    }
}

void mtsRobotIO1394::RobotInternal::DriveAmpsToBits(const vctDoubleVec &fromData, vctLongVec &toData) const
{
    toData.SetAll(0L);
    for (size_t index=0; index<JointList.size();index++){
        double ampToBitsScale = JointList[index].drive.AmpsToBitsScale;
        double ampToBitsOffset = JointList[index].drive.AmpsToBitsOffset;
        double maxAmps = JointList[index].drive.MaxCurrentValue;

        double tmpValue;
        if(fromData[index] > maxAmps){
            tmpValue = maxAmps;
        }
        else if(fromData[index] < -maxAmps){
            tmpValue = -maxAmps;
        }
        else{
            tmpValue = fromData[index];
        }
        toData[index] = (tmpValue * ampToBitsScale) + ampToBitsOffset;
    }
}

void mtsRobotIO1394::RobotInternal::DriveBitsToFbAmps(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index=0; index<JointList.size();index++){
        double bitsToFbAmpsScale = JointList[index].drive.BitsToFbAmpsScale;
        double bitsToFbAmpsOffset = JointList[index].drive.BitsToFbAmpsOffset;

        toData[index] = (fromData[index] * bitsToFbAmpsScale) + bitsToFbAmpsOffset;
    }
}

void mtsRobotIO1394::RobotInternal::DriveNmToAmps(const vctDoubleVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0L);
    for (size_t index=0; index<JointList.size(); index++){
        double nmToAmps = JointList[index].drive.NmToAmpsScale;

        toData[index] = fromData[index]*nmToAmps;
    }
}

void mtsRobotIO1394::RobotInternal::DriveAmpsToNm(const vctDoubleVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index=0; index<JointList.size(); index++){
        double nmToAmps = JointList[index].drive.NmToAmpsScale;

        toData[index] = fromData[index] / nmToAmps;
    }
}

void mtsRobotIO1394::RobotInternal::AnalogInBitsToVolts(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index=0; index<JointList.size(); index++){
        double bitsToVoltsScale = JointList[index].analogIn.BitsToVoltsScale;
        double bitsToVoltsOffset = JointList[index].analogIn.BitsToVoltsOffset;

        toData[index] = (fromData[index] * bitsToVoltsScale) + bitsToVoltsOffset;
    }
}

void mtsRobotIO1394::RobotInternal::AnalogInVoltsToPosSI(const vctDoubleVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index=0; index<JointList.size(); index++){
        double voltsToPosSIScale = JointList[index].analogIn.VoltsToPosSIScale;
        double voltsToPosSIOffset = JointList[index].analogIn.VoltsToPosSIOffset;

        toData[index] = (fromData[index]  * voltsToPosSIScale) + voltsToPosSIOffset;
    }
}

//Future Works:
//Variable config file with ENC/POT. Set flags so Get/Set is properly configured. Configure Function should account for different
//Layouts.
