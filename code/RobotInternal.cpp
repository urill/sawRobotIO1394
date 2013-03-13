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



#include "RobotInternal.h"
#include "AmpIO.h"

const unsigned long WD_MSTOCOUNT =       192;    // watchdog counts per ms (note counter width, e.g. 16 bits)

// ActuatorInfo Constructor
mtsRobotIO1394::RobotInternal::ActuatorInfo::ActuatorInfo() : board(0), axisid(-1)
{}

mtsRobotIO1394::RobotInternal::ActuatorInfo::ActuatorInfo(AmpIO *bptr, int aid): board(bptr), axisid(aid)
{}

// ActuatorInfo Destructor
mtsRobotIO1394::RobotInternal::ActuatorInfo::~ActuatorInfo()
{}

mtsRobotIO1394::RobotInternal::RobotInternal(const std::string &name, size_t numActuators) :
    robotName(name), ActuatorList(numActuators), valid(false),
    ampStatus(numActuators, false), ampEnable(numActuators, false),
    encPosRaw(numActuators), encPos(numActuators),
    encVelRaw(numActuators), encVel(numActuators),
    analogInRaw(numActuators), analogInVolts(numActuators), analogInPosSI(numActuators),
    motorFeedbackCurrentRaw(numActuators), motorFeedbackCurrent(numActuators),
    motorControlCurrentRaw(numActuators), motorControlCurrent(numActuators)
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

    Configure(xmlConfig, 1);
}

void mtsRobotIO1394::RobotInternal::Configure (cmnXMLPath  &xmlConfigFile, int robotNumber){
    char path[64];
    std::string context = "Config";

    int  tmpNumOfActuator;
    sprintf(path, "Robot[%i]/@NumOfActuator", robotNumber);
    xmlConfigFile.GetXMLValue(context.c_str(), path, tmpNumOfActuator);

    for(int i=0; i<tmpNumOfActuator; i++)    {
        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/AmpsToBits/@Scale", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.AmpsToBitsScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/AmpsToBits/@Offset", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.AmpsToBitsOffset);

        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/BitsToFbAmps/@Scale", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.BitsToFbAmpsScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/BitsToFbAmps/@Offset", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.BitsToFbAmpsOffset);

        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/NmToAmps/@Scale", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.NmToAmpsScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/MaxCurrent/@Value", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.MaxCurrentValue);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToPosSI/@Scale", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToPosSIScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToPosSI/@Offset", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToPosSIOffset);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaPosSI/@Scale", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToDeltaPosSIScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaPosSI/@Offset", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToDeltaPosSIOffset);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaT/@Scale", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToDeltaTScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaT/@Offset", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToDeltaTOffset);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/CountsPerTurn/@Value", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.CountsPerTurnValue);

        sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/BitsToVolts/@Scale", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].analogIn.BitsToVoltsScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/BitsToVolts/@Offset", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].analogIn.BitsToVoltsOffset);

        sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/VoltsToPosSI/@Scale", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].analogIn.VoltsToPosSIScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/VoltsToPosSI/@Offset", robotNumber, i+1);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].analogIn. VoltsToPosSIOffset);
    }

    ConfigureCoupling (xmlConfigFile, robotNumber, couplingStatus);
}

void mtsRobotIO1394::RobotInternal::ConfigureCoupling (cmnXMLPath &xmlConfigFile, int robotNumber, bool &couplingEnable){
    char path[64];
    std::string context = "Config";
    int tmpNumOfActuator = 0;
    int tmpNumOfJoint = 0;

    bool tmpCouplingAvailable=false;
    int buff=0;
    sprintf(path,"Robot[%i]/Coupling/@Value",robotNumber);
    xmlConfigFile.GetXMLValue(context.c_str(),path,buff);

    if(buff==1) {
        //The Coupling Value must be equal to 1 for this configuration to work.
        tmpCouplingAvailable=true;
    }
    else {
        tmpCouplingAvailable=false;
    }
    tmpCouplingAvailable=true;
    if(tmpCouplingAvailable) {
        sprintf(path, "Robot[%i]/@NumOfActuator", robotNumber);
        xmlConfigFile.GetXMLValue(context.c_str(),path,tmpNumOfActuator);
        sprintf(path, "Robot[%i]/@NumOfJoint", robotNumber);
        xmlConfigFile.GetXMLValue(context.c_str(),path,tmpNumOfJoint);
        actuatorToJoint.SetSize(tmpNumOfJoint,tmpNumOfActuator,0.0);
        jointToActuator.SetSize(tmpNumOfActuator,tmpNumOfJoint,0.0);
        actTorqueToJointTorque.SetSize(tmpNumOfJoint,tmpNumOfActuator,0.0);
        jointTorqueToActTorque.SetSize(tmpNumOfActuator,tmpNumOfJoint,0.0);

        ConfigureCouplingA2J(xmlConfigFile,robotNumber,tmpNumOfActuator,tmpNumOfJoint,actuatorToJoint);
        ConfigureCouplingJ2A(xmlConfigFile,robotNumber,tmpNumOfActuator,tmpNumOfJoint,jointToActuator);
        ConfigureCouplingAT2JT(xmlConfigFile,robotNumber,tmpNumOfActuator,tmpNumOfJoint,actTorqueToJointTorque);
        ConfigureCouplingJT2AT(xmlConfigFile,robotNumber,tmpNumOfActuator,tmpNumOfJoint,jointTorqueToActTorque);
        //Still need to do proper alignment and such for joint/actuator situ for each matrix.
    }
    couplingEnable = tmpCouplingAvailable;
}

void mtsRobotIO1394::RobotInternal::ConfigureCouplingA2J (cmnXMLPath &xmlConfigFile,
                                                          int robotNumber, int numOfActuator,
                                                          int numOfJoint, vctDoubleMat &A2JMatrix) {
    char path[64];
    std::string context = "Config";
    std::string tmpRow = "";
    size_t i=0;

    for (i=0; i<numOfJoint;i++) {
        vctDoubleVec tmpRowVec;
        tmpRowVec.SetSize(numOfActuator);

        std::stringstream tmpStringStream;
        sprintf(path,"Robot[%i]/Coupling/ActuatorToJoint/Row[%i]/@Val",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,tmpRow);
        tmpStringStream.str (tmpRow);

        tmpRowVec.FromStreamRaw(tmpStringStream);
        A2JMatrix.Row(i).Assign(tmpRowVec);
    }
    //std::cout<<"A2JMatrix: "<<std::endl<<A2JMatrix<<std::endl<<std::endl;
}

void mtsRobotIO1394::RobotInternal::ConfigureCouplingJ2A (cmnXMLPath &xmlConfigFile,
                                                          int robotNumber, int numOfActuator,
                                                          int numOfJoint, vctDoubleMat &J2AMatrix) {
    char path[64];
    std::string context = "Config";
    std::string tmpRow = "";
    size_t i=0;

    for (i=0; i<numOfJoint;i++) {
        vctDoubleVec tmpRowVec;
        tmpRowVec.SetSize(numOfActuator);
        std::stringstream tmpStringStream;
        sprintf(path,"Robot[%i]/Coupling/JointToActuator/Row[%i]/@Val",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,tmpRow);
        tmpStringStream.str (tmpRow);

        tmpRowVec.FromStreamRaw(tmpStringStream);

        J2AMatrix.Row(i).Assign(tmpRowVec);
    }
    //std::cout<<"J2AMatrix: "<<std::endl<<J2AMatrix<<std::endl<<std::endl;
}

void mtsRobotIO1394::RobotInternal::ConfigureCouplingAT2JT (cmnXMLPath &xmlConfigFile,
                                                          int robotNumber, int numOfActuator,
                                                          int numOfJoint, vctDoubleMat &AT2JTMatrix) {
    char path[64];
    std::string context = "Config";
    std::string tmpRow = "";
    size_t i=0;

    for (i=0; i<numOfJoint;i++) {
        vctDoubleVec tmpRowVec;
        tmpRowVec.SetSize(numOfActuator);

        std::stringstream tmpStringStream;
        sprintf(path,"Robot[%i]/Coupling/ActuatorTorqueToJointTorque/Row[%i]/@Val",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,tmpRow);
        tmpStringStream.str (tmpRow);

        tmpRowVec.FromStreamRaw(tmpStringStream);

        AT2JTMatrix.Row(i).Assign(tmpRowVec);
    }
}

void mtsRobotIO1394::RobotInternal::ConfigureCouplingJT2AT (cmnXMLPath &xmlConfigFile,
                                                          int robotNumber, int numOfActuator,
                                                          int numOfJoint, vctDoubleMat &JT2ATMatrix) {
    char path[64];
    std::string context = "Config";
    std::string tmpRow = "";
    size_t i=0;

    for (i=0; i<numOfJoint;i++) {
        vctDoubleVec tmpRowVec;
        tmpRowVec.SetSize(numOfActuator);

        std::stringstream tmpStringStream;
        sprintf(path,"Robot[%i]/Coupling/JointTorqueToActuatorTorque/Row[%i]/@Val",robotNumber,i+1);
        xmlConfigFile.GetXMLValue(context.c_str(),path,tmpRow);
        tmpStringStream.str (tmpRow);

        tmpRowVec.FromStreamRaw(tmpStringStream);
        JT2ATMatrix.Row(i).Assign(tmpRowVec);
    }
}

void mtsRobotIO1394::RobotInternal::SetActuatorInfo(int index, AmpIO *board, int axis)
{
    ActuatorList[index] = ActuatorInfo(board, axis);
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
    stateTable.AddData(digitalIn, robotName + "DigitalIn");
    stateTable.AddData(motorFeedbackCurrentRaw, robotName + "MotorFeedbackCurrentRaw");
    stateTable.AddData(motorFeedbackCurrent, robotName + "MotorFeedbackCurrent");
    stateTable.AddData(motorControlCurrentRaw, robotName + "MotorControlCurrentRaw");
    stateTable.AddData(motorControlCurrent, robotName + "MotorControlCurrent");
}

void mtsRobotIO1394::RobotInternal::SetupProvidedInterface(mtsInterfaceProvided *prov, mtsStateTable &stateTable)
{
    prov->AddCommandRead(&mtsRobotIO1394::RobotInternal::GetNumberOfActuators, this,
                         "GetNumberOfActuators");
    prov->AddCommandReadState(stateTable, this->valid, "IsValid");

    // Enable // Disable
    prov->AddCommandVoid(&mtsRobotIO1394::RobotInternal::EnablePower, this, "EnablePower");
    prov->AddCommandVoid(&mtsRobotIO1394::RobotInternal::DisablePower, this, "DisablePower");
    prov->AddCommandVoid(&mtsRobotIO1394::RobotInternal::EnableSafetyRelay, this, "EnableSafetyRelay");
    prov->AddCommandVoid(&mtsRobotIO1394::RobotInternal::DisableSafetyRelay, this, "DisableSafetyRelay");
    prov->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetAmpEnable, this, "SetAmpEnable",
                          this->ampEnable);

    prov->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetWatchdogPeriod, this, "SetWatchdogPeriod",
                          this->watchdogPeriod);

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

    prov->AddCommandReadState(stateTable, this->digitalIn, "GetDigitalInput");

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
                                  "AnalogInBitsToVolts", analogInRaw, analogInVolts);
    prov->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::AnalogInVoltsToPosSI, this,
                                  "AnalogInVoltsToPosSI", analogInVolts, analogInPosSI);
}

bool mtsRobotIO1394::RobotInternal::CheckIfValid(void)
{
    size_t i;
    for (i = 0; i < ActuatorList.size(); i++) {
        ActuatorInfo &jt = ActuatorList[i];
        if (!jt.board || (jt.axisid < 0)) break;  // should not happen
        if (!jt.board->ValidRead()) break;
    }
    valid = (i == ActuatorList.size());
    return valid;
}

void mtsRobotIO1394::RobotInternal::GetData(void)
{
    powerStatus = true;
    safetyRelay = true;
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        AmpIO *board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
        if (!board || (axis < 0)) continue;
        encPosRaw[index] = board->GetEncoderPosition(axis);
        encVelRaw[index] = board->GetEncoderVelocity(axis);
        analogInRaw[index] = board->GetAnalogInput(axis);
        // digitalIn[index] = board->GetDigitalInput(axis);
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

void mtsRobotIO1394::RobotInternal::GetNumberOfActuators(int &num) const
{
    num = ActuatorList.size();
}

void mtsRobotIO1394::RobotInternal::EnablePower(void)
{
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        AmpIO *board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
        if (!board || (axis < 0)) continue;
        // Make sure all boards are enabled
        board->SetPowerEnable(true);
        // For now, also enable all amplifiers
        board->SetAmpEnable(axis, true);
    }
}

void mtsRobotIO1394::RobotInternal::DisablePower(void)
{
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        AmpIO *board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
        if (!board || (axis < 0)) continue;
        // Make sure all boards are disabled
        board->SetPowerEnable(false);
        // For now, also disable all amplifiers
        board->SetAmpEnable(axis, false);
    }
}

void mtsRobotIO1394::RobotInternal::EnableSafetyRelay(void)
{
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        AmpIO *board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
        if (!board || (axis < 0)) continue;
        board->SetSafetyRelay(true);
    }
}

void mtsRobotIO1394::RobotInternal::DisableSafetyRelay(void)
{
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        AmpIO *board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
        if (!board || (axis < 0)) continue;
        board->SetSafetyRelay(false);
    }
}

void mtsRobotIO1394::RobotInternal::SetWatchdogPeriod(const unsigned long &period_ms)
{
    // assume MAX_BOARDS < 255
    vctUCharVec board_list(BoardIO::MAX_BOARDS, (unsigned char)0xff);
    watchdogPeriod = period_ms;

    // TODO: a more direct way of accessing the board list from this class?
    for (size_t index = 0; index < ActuatorList.size(); index++)
    {
        // get board associated with each joint
        AmpIO *board = ActuatorList[index].board;
        if (!board || !board->IsValid()) continue;

        // check board id to skip boards already written to
        unsigned char board_id = board->GetBoardId();
        if (board_list[board_id] == board_id) continue;
        board_list[board_id] = board_id;

        // write timeout period, converted to counts
        board->WriteWatchdogPeriod(period_ms*WD_MSTOCOUNT);
    }
}

void mtsRobotIO1394::RobotInternal::SetAmpEnable(const vctBoolVec &ampControl)
{
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        AmpIO *board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
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
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        AmpIO *board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
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
    for (size_t index = 0; index<ActuatorList.size();index++) {
        double bitsToPosSIScale = ActuatorList[index].encoder.BitsToPosSIScale;
        double bitsToPosSIOffset = ActuatorList[index].encoder.BitsToPosSIOffset;
        toData[index] = (fromData[index] * bitsToPosSIScale) + bitsToPosSIOffset;
    }
}

void mtsRobotIO1394::RobotInternal::EncoderSIToRaw(const vctDoubleVec &fromData, vctLongVec &toData) const
{
    toData.SetAll(0L);
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        double bitsToPosSIScale = ActuatorList[index].encoder.BitsToPosSIScale;
        double bitsToPosSIOffset = ActuatorList[index].encoder.BitsToPosSIOffset;
        toData[index] = (fromData[index]-bitsToPosSIOffset) / bitsToPosSIScale;
    }
}

void mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosSI(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index = 0; index<ActuatorList.size();index++) {
        double bitsToDeltaPosSIScale = ActuatorList[index].encoder.BitsToDeltaPosSIScale;
        double bitsToDeltaPosSIOffset = ActuatorList[index].encoder.BitsToDeltaPosSIOffset;
        toData[index] = (fromData[index] * bitsToDeltaPosSIScale) + bitsToDeltaPosSIOffset;
    }
}

void mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosT(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index = 0; index<ActuatorList.size();index++) {
        double bitsToDeltaTScale = ActuatorList[index].encoder.BitsToDeltaTScale;
        double bitsToDeltaTOffset = ActuatorList[index].encoder.BitsToDeltaTOffset;
        toData[index] = (fromData[index] * bitsToDeltaTScale) + bitsToDeltaTOffset ;
    }
}

void mtsRobotIO1394::RobotInternal::DriveAmpsToBits(const vctDoubleVec &fromData, vctLongVec &toData) const
{
    toData.SetAll(0L);
    for (size_t index=0; index<ActuatorList.size();index++) {
        double ampToBitsScale = ActuatorList[index].drive.AmpsToBitsScale;
        double ampToBitsOffset = ActuatorList[index].drive.AmpsToBitsOffset;
        double maxAmps = ActuatorList[index].drive.MaxCurrentValue;
        double tmpValue;
        if (fromData[index] > maxAmps) {
            tmpValue = maxAmps;
        }
        else if (fromData[index] < -maxAmps) {
            tmpValue = -maxAmps;
        }
        else {
            tmpValue = fromData[index];
        }
        toData[index] = (tmpValue * ampToBitsScale) + ampToBitsOffset;
    }
}

void mtsRobotIO1394::RobotInternal::DriveBitsToFbAmps(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index=0; index<ActuatorList.size();index++){
        double bitsToFbAmpsScale = ActuatorList[index].drive.BitsToFbAmpsScale;
        double bitsToFbAmpsOffset = ActuatorList[index].drive.BitsToFbAmpsOffset;
        toData[index] = (fromData[index] * bitsToFbAmpsScale) + bitsToFbAmpsOffset;
    }
}

void mtsRobotIO1394::RobotInternal::DriveNmToAmps(const vctDoubleVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0L);
    for (size_t index=0; index<ActuatorList.size(); index++) {
        double nmToAmps = ActuatorList[index].drive.NmToAmpsScale;
        toData[index] = fromData[index]*nmToAmps;
    }
}

void mtsRobotIO1394::RobotInternal::DriveAmpsToNm(const vctDoubleVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index=0; index<ActuatorList.size(); index++) {
        double nmToAmps = ActuatorList[index].drive.NmToAmpsScale;
        toData[index] = fromData[index] / nmToAmps;
    }
}

void mtsRobotIO1394::RobotInternal::AnalogInBitsToVolts(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index=0; index<ActuatorList.size(); index++) {
        double bitsToVoltsScale = ActuatorList[index].analogIn.BitsToVoltsScale;
        double bitsToVoltsOffset = ActuatorList[index].analogIn.BitsToVoltsOffset;
        toData[index] = (fromData[index] * bitsToVoltsScale) + bitsToVoltsOffset;
    }
}

void mtsRobotIO1394::RobotInternal::AnalogInVoltsToPosSI(const vctDoubleVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    for (size_t index=0; index<ActuatorList.size(); index++) {
        double voltsToPosSIScale = ActuatorList[index].analogIn.VoltsToPosSIScale;
        double voltsToPosSIOffset = ActuatorList[index].analogIn.VoltsToPosSIOffset;

        toData[index] = (fromData[index]  * voltsToPosSIScale) + voltsToPosSIOffset;
    }
}

//Future Works:
//Variable config file with ENC/POT. Set flags so Get/Set is properly configured. Configure Function should account for different
//Layouts.
