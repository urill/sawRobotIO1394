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

// ZC: MOVE to configuration file
// const unsigned int      ENC_CPT  =      4000;    // OK  1000 x 4 quadrature
// const unsigned long ENC_VEL_MAX  =  0x00FFFF;    // TEMP need check // maximum value of encoder pulse period
// const double        ENC_VEL_CLK  = 1000000.0;    // TMEP need check clock (Hz) used to measure encoder pulse
// const double        ENC_ACC_CLK  =      12.0;    // TEMP need check
// const unsigned long MIDRANGE_ADC = 0x0008000;    // 16 bits ADC mid range value
const unsigned long ENC_OFFSET   = 0x007FFFFF;   // Encoder offset
const unsigned long WD_MSTOCOUNT =       192;    // watchdog counts per ms (note counter width, e.g. 16 bits)

// ActuatorInfo Constructor
mtsRobotIO1394::RobotInternal::ActuatorInfo::ActuatorInfo() : board(0), axisid(-1)
{}

mtsRobotIO1394::RobotInternal::ActuatorInfo::ActuatorInfo(AmpIO *bptr, int aid): board(bptr), axisid(aid)
{}

// ActuatorInfo Destructor
mtsRobotIO1394::RobotInternal::ActuatorInfo::~ActuatorInfo()
{}

mtsRobotIO1394::RobotInternal::RobotInternal(const std::string & name,
                                             const cmnGenericObject & owner,
                                             size_t numActuators) :
    OwnerServices(owner.Services()),
    robotName(name), ActuatorList(numActuators), Valid(false),
    ampStatus(numActuators, false), ampEnable(numActuators, false),
    encPosRaw(numActuators), PositionJoint(numActuators),
    PositionJointGet(numActuators), PositionActuatorGet(numActuators),
    encVelRaw(numActuators), encVel(numActuators),
    analogInRaw(numActuators), analogInVolts(numActuators), analogInPosSI(numActuators),
    motorFeedbackCurrentRaw(numActuators), motorFeedbackCurrent(numActuators),
    TorqueJoint(numActuators),
    motorControlCurrentRaw(numActuators), motorControlCurrent(numActuators),
    encSetPosRaw(numActuators), encSetPos(numActuators)
{
}

mtsRobotIO1394::RobotInternal::~RobotInternal()
{
}
void mtsRobotIO1394::RobotInternal::Configure(const std::string & filename){
    //This configuration will go by default method. Will assume there is only one robot, and the first one is it.
    //If there are more than one robot per configuration file, then this method should not be used.

    cmnXMLPath xmlConfig;
    xmlConfig.SetInputSource(filename);

    Configure(xmlConfig, 1);
}

void mtsRobotIO1394::RobotInternal::Configure (cmnXMLPath  & xmlConfigFile, int robotNumber){
    char path[64];
    std::string context = "Config";

    int  tmpNumOfActuator;
    sprintf(path, "Robot[%i]/@NumOfActuator", robotNumber);
    xmlConfigFile.GetXMLValue(context.c_str(), path, tmpNumOfActuator);
    
    int xmlIndex;

    for (int i = 0; i < tmpNumOfActuator; i++) {
        xmlIndex = i + 1;

        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/AmpsToBits/@Scale", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.AmpsToBitsScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/AmpsToBits/@Offset", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.AmpsToBitsOffset);

        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/BitsToFeedbackAmps/@Scale", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.BitsToFeedbackAmpsScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/BitsToFeedbackAmps/@Offset", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.BitsToFeedbackAmpsOffset);

        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/NmToAmps/@Scale", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.NmToAmpsScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/Drive/MaxCurrent/@Value", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].drive.MaxCurrentValue);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToPosSI/@Scale", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToPosSIScale);
        ActuatorList[i].encoder.BitsToPosSIScale *= cmnPI_180; // -------------------------------------------- adeguet1, make sure these are degrees

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToPosSI/@Offset", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToPosSIOffset);
        ActuatorList[i].encoder.BitsToPosSIOffset *= cmnPI_180; // -------------------------------------------- adeguet1, make sure these are degrees

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaPosSI/@Scale", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToDeltaPosSIScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaPosSI/@Offset", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToDeltaPosSIOffset);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaT/@Scale", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToDeltaTScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaT/@Offset", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.BitsToDeltaTOffset);

        sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/CountsPerTurn/@Value", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].encoder.CountsPerTurnValue);

        sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/BitsToVolts/@Scale", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].analogIn.BitsToVoltsScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/BitsToVolts/@Offset", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].analogIn.BitsToVoltsOffset);

        sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/VoltsToPosSI/@Scale", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].analogIn.VoltsToPosSIScale);

        sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/VoltsToPosSI/@Offset", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].analogIn. VoltsToPosSIOffset);
    }

    ConfigureCoupling(xmlConfigFile, robotNumber);
    // PreloadEncoders();
}

void
mtsRobotIO1394::RobotInternal::ConfigureCoupling(cmnXMLPath & xmlConfigFile, int robotNumber) {
    char path[64];
    std::string context = "Config";
    int tmpNumOfActuator = 0;
    int tmpNumOfJoint = 0;

    bool tmpCouplingAvailable = false;
    int buff = 0;
    sprintf(path, "Robot[%i]/Coupling/@Value", robotNumber);
    xmlConfigFile.GetXMLValue(context.c_str(), path, buff);

    if (buff == 1) {
        //The Coupling Value must be equal to 1 for this configuration to work.
        tmpCouplingAvailable = true;
    }
    else {
        tmpCouplingAvailable = false;
    }
    tmpCouplingAvailable = true;
    if (tmpCouplingAvailable) {
        sprintf(path, "Robot[%i]/@NumOfActuator", robotNumber);
        xmlConfigFile.GetXMLValue(context.c_str(), path, tmpNumOfActuator);
        sprintf(path, "Robot[%i]/@NumOfJoint", robotNumber);
        xmlConfigFile.GetXMLValue(context.c_str(), path, tmpNumOfJoint);
        ActuatorToJointPosition.SetSize(tmpNumOfJoint, tmpNumOfActuator, 0.0);
        JointToActuatorPosition.SetSize(tmpNumOfActuator, tmpNumOfJoint, 0.0);
        ActuatorToJointTorque.SetSize(tmpNumOfJoint, tmpNumOfActuator, 0.0);
        JointToActuatorTorque.SetSize(tmpNumOfActuator, tmpNumOfJoint, 0.0);

        ConfigureCouplingA2J(xmlConfigFile, robotNumber, tmpNumOfActuator, tmpNumOfJoint, ActuatorToJointPosition);
        ConfigureCouplingJ2A(xmlConfigFile, robotNumber, tmpNumOfActuator, tmpNumOfJoint, JointToActuatorPosition);
        ConfigureCouplingAT2JT(xmlConfigFile, robotNumber, tmpNumOfActuator, tmpNumOfJoint, ActuatorToJointTorque);
        ConfigureCouplingJT2AT(xmlConfigFile, robotNumber, tmpNumOfActuator, tmpNumOfJoint, JointToActuatorTorque);
        //Still need to do proper alignment and such for joint/actuator situ for each matrix.
    }
    this->HasActuatorToJointCoupling = tmpCouplingAvailable;
    this->NumberOfJoints = tmpNumOfJoint;
    std::cerr << "ConfigureCoupling: product of actuator to joint position coupling matrices (should be close to identity)" << std::endl
              << this->ActuatorToJointPosition * this->JointToActuatorPosition << std::endl;
    std::cerr << "ConfigureCoupling: product of actuator to joint torque coupling matrices (should be close to identity)" << std::endl
              << this->ActuatorToJointTorque * this->JointToActuatorTorque << std::endl;

}

void mtsRobotIO1394::RobotInternal::ConfigureCouplingA2J (cmnXMLPath & xmlConfigFile,
                                                          int robotNumber, int numOfActuator,
                                                          int numOfJoint, vctDoubleMat & A2JMatrix) {
    std::string tmpPathString = "";
    char path[64];
    sprintf(path, "Robot[%i]/Coupling/ActuatorToJointPosition", robotNumber);
    tmpPathString = path;
    ConfigureCouplingMatrix(xmlConfigFile, tmpPathString, numOfJoint, numOfActuator, A2JMatrix);
}

void mtsRobotIO1394::RobotInternal::ConfigureCouplingJ2A (cmnXMLPath & xmlConfigFile,
                                                          int robotNumber, int numOfActuator,
                                                          int numOfJoint, vctDoubleMat & J2AMatrix) {
    std::string tmpPathString = "";
    char path[64];
    sprintf(path, "Robot[%i]/Coupling/JointToActuatorPosition", robotNumber);
    tmpPathString = path;
    ConfigureCouplingMatrix(xmlConfigFile, tmpPathString, numOfActuator, numOfJoint, J2AMatrix);
}

void mtsRobotIO1394::RobotInternal::ConfigureCouplingAT2JT (cmnXMLPath & xmlConfigFile,
                                                          int robotNumber, int numOfActuator,
                                                          int numOfJoint, vctDoubleMat & AT2JTMatrix) {
    std::string tmpPathString = "";
    char path[64];
    sprintf(path, "Robot[%i]/Coupling/ActuatorToJointTorque", robotNumber);
    tmpPathString = path;
    ConfigureCouplingMatrix(xmlConfigFile, tmpPathString, numOfJoint, numOfActuator, AT2JTMatrix);
}

void mtsRobotIO1394::RobotInternal::ConfigureCouplingJT2AT (cmnXMLPath & xmlConfigFile,
                                                          int robotNumber, int numOfActuator,
                                                          int numOfJoint, vctDoubleMat & JT2ATMatrix) {
    std::string tmpPathString = "";
    char path[64];
    sprintf(path, "Robot[%i]/Coupling/JointToActuatorTorque", robotNumber);
    tmpPathString = path;
    ConfigureCouplingMatrix(xmlConfigFile, tmpPathString, numOfActuator, numOfJoint, JT2ATMatrix);
}

void mtsRobotIO1394::RobotInternal::ConfigureCouplingMatrix (cmnXMLPath & xmlConfigFile, const std::string pathToMatrix,
                                                             int numRows, int numCols, vctDoubleMat & resultMatrix) {
    char path[64];

    std::string context = "Config";
    std::string tmpRow = "";
    bool ssTest = false;
    size_t i = 0;

    for (i = 0; i < numRows; i++) {
        vctDoubleVec tmpRowVec;
        tmpRowVec.SetSize(numCols);
        std::stringstream tmpStringStream;
        tmpRow = "";
        ssTest = false;
        char tmpPath[10];
        sprintf(tmpPath, "/Row[%i]/@Val", i + 1);
        strcpy(path, pathToMatrix.c_str());
        strcat(path, tmpPath);
        xmlConfigFile.GetXMLValue(context.c_str(), path, tmpRow);
        tmpStringStream.str(tmpRow);
        ssTest = tmpRowVec.FromStreamRaw(tmpStringStream);
        if (!ssTest) {
            CMN_LOG_CLASS_INIT_ERROR << "Row vector Assign failed on row " << i << ", path: " << pathToMatrix << std::endl;
        }
        resultMatrix.Row(i).Assign(tmpRowVec);
    }
}

void mtsRobotIO1394::RobotInternal::SetActuatorInfo(int index, AmpIO * board, int axis)
{
    ActuatorList[index] = ActuatorInfo(board, axis);
}

void mtsRobotIO1394::RobotInternal::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(Valid, robotName + "Valid");
    stateTable.AddData(PowerStatus, robotName + "PowerStatus");
    stateTable.AddData(SafetyRelay, robotName + "SafetyRelay");
    stateTable.AddData(ampStatus, robotName + "AmpStatus");
    stateTable.AddData(ampEnable, robotName + "AmpEnable");
    stateTable.AddData(encPosRaw, robotName + "PosRaw");
    stateTable.AddData(PositionJoint, robotName + "PositionJoint");
    stateTable.AddData(PositionJointGet, robotName + "PositionJointGet");
    stateTable.AddData(PositionActuatorGet, robotName + "PositionActuatorGet");
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

void mtsRobotIO1394::RobotInternal::SetupInterfaces(mtsInterfaceProvided * robotInterface,
                                                    mtsInterfaceProvided * actuatorInterface,
                                                    mtsStateTable & stateTable)
{
    robotInterface->AddCommandRead(& mtsRobotIO1394::RobotInternal::GetNumberOfActuators, this,
                                   "GetNumberOfActuators");
    robotInterface->AddCommandRead(& mtsRobotIO1394::RobotInternal::GetNumberOfJoints, this,
                                   "GetNumberOfJoints");
    robotInterface->AddCommandReadState(stateTable, this->Valid, "IsValid");

    // Enable // Disable
    robotInterface->AddCommandVoid(& mtsRobotIO1394::RobotInternal::EnablePower, this, "EnablePower");
    robotInterface->AddCommandVoid(& mtsRobotIO1394::RobotInternal::DisablePower, this, "DisablePower");
    robotInterface->AddCommandVoid(& mtsRobotIO1394::RobotInternal::EnableSafetyRelay, this, "EnableSafetyRelay");
    robotInterface->AddCommandVoid(& mtsRobotIO1394::RobotInternal::DisableSafetyRelay, this, "DisableSafetyRelay");

    robotInterface->AddCommandWrite(& mtsRobotIO1394::RobotInternal::SetWatchdogPeriod, this, "SetWatchdogPeriod",
                                    this->watchdogPeriod);

    robotInterface->AddCommandReadState(stateTable, this->PowerStatus, "GetPowerStatus"); // bool
    robotInterface->AddCommandReadState(stateTable, this->SafetyRelay, "GetSafetyRelay"); // unsigned short

    robotInterface->AddCommandReadState(stateTable, this->encPosRaw, "GetPositionEncoderRaw"); // vector[int]
    robotInterface->AddCommandReadState(stateTable, this->PositionJoint, "GetPosition"); // vector[double]

    robotInterface->AddCommandReadState(stateTable, this->PositionJointGet, "GetPositionJoint"); // prmPositionJointGet

    robotInterface->AddCommandReadState(stateTable, this->encVelRaw, "GetVelocityRaw");
    robotInterface->AddCommandReadState(stateTable, this->encVel, "GetVelocity");

    robotInterface->AddCommandReadState(stateTable, this->analogInRaw, "GetAnalogInputRaw");
    robotInterface->AddCommandReadState(stateTable, this->analogInVolts, "GetAnalogInputVolts");
    robotInterface->AddCommandReadState(stateTable, this->analogInPosSI, "GetAnalogInputPosSI");

    robotInterface->AddCommandReadState(stateTable, this->digitalIn, "GetDigitalInput");

    robotInterface->AddCommandReadState(stateTable, this->motorFeedbackCurrentRaw, "GetMotorFeedbackCurrentRaw");
    robotInterface->AddCommandReadState(stateTable, this->motorFeedbackCurrent, "GetMotorFeedbackCurrent");

    robotInterface->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetTorqueJoint, this, "SetTorqueJoint", TorqueJoint);
    robotInterface->AddCommandWrite(& mtsRobotIO1394::RobotInternal::SetMotorCurrentRaw, this, "SetMotorCurrentRaw",
                                    motorControlCurrentRaw);
    robotInterface->AddCommandWrite(& mtsRobotIO1394::RobotInternal::SetMotorCurrent, this, "SetMotorCurrent",
                                    motorControlCurrent);

    robotInterface->AddCommandWrite(& mtsRobotIO1394::RobotInternal::SetEncoderPositionRaw, this, "SetEncoderPositionRaw",
                                    encSetPosRaw);
    robotInterface->AddCommandWrite(& mtsRobotIO1394::RobotInternal::SetEncoderPosition, this, "SetEncoderPosition",
                                    encSetPos);

    // unit conversion methods (Qualified Read)
    robotInterface->AddCommandQualifiedRead(& mtsRobotIO1394::RobotInternal::EncoderRawToSI, this,
                                            "EncoderRawToSI", encPosRaw, vctDoubleVec());
    robotInterface->AddCommandQualifiedRead(& mtsRobotIO1394::RobotInternal::EncoderSIToRaw, this,
                                            "EncoderSIToRaw", vctDoubleVec(), encPosRaw);
    robotInterface->AddCommandQualifiedRead(& mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosSI, this,
                                            "EncoderRawToDeltaPosSI", encVelRaw, encVel);
    robotInterface->AddCommandQualifiedRead(& mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosT, this,
                                            "EncoderRawToDeltaPosT", encVelRaw, encVel);
    robotInterface->AddCommandQualifiedRead(& mtsRobotIO1394::RobotInternal::DriveAmpsToNm, this,
                                            "DriveAmpsToNm", motorControlCurrent, motorControlTorque);
    robotInterface->AddCommandQualifiedRead(& mtsRobotIO1394::RobotInternal::DriveNmToAmps, this,
                                            "DriveNmToAmps", motorControlTorque, motorControlCurrent);
    robotInterface->AddCommandQualifiedRead(& mtsRobotIO1394::RobotInternal::AnalogInBitsToVolts, this,
                                            "AnalogInBitsToVolts", analogInRaw, analogInVolts);

    actuatorInterface->AddCommandWrite(& mtsRobotIO1394::RobotInternal::SetAmpEnable, this, "SetAmpEnable",
                                       this->ampEnable); // vector[bool]
    actuatorInterface->AddCommandWrite(& mtsRobotIO1394::RobotInternal::ResetSingleEncoder, this, "ResetSingleEncoder"); // int
    actuatorInterface->AddCommandReadState(stateTable, this->ampEnable, "GetAmpEnable"); // vector[bool]
    actuatorInterface->AddCommandReadState(stateTable, this->ampStatus, "GetAmpStatus"); // vector[bool]

    actuatorInterface->AddCommandReadState(stateTable, this->PositionActuatorGet, "GetPositionActuator"); // prmPositionJointGet

    actuatorInterface->AddCommandQualifiedRead(& mtsRobotIO1394::RobotInternal::DriveAmpsToBits, this,
                                               "DriveAmpsToBits", motorFeedbackCurrent, motorFeedbackCurrentRaw);

    actuatorInterface->AddCommandQualifiedRead(& mtsRobotIO1394::RobotInternal::AnalogInVoltsToPosSI, this,
                                               "AnalogInVoltsToPosSI", analogInVolts, analogInPosSI);

}

bool mtsRobotIO1394::RobotInternal::CheckIfValid(void)
{
    size_t i;
    for (i = 0; i < ActuatorList.size(); i++) {
        ActuatorInfo & jt = ActuatorList[i];
        if (!jt.board || (jt.axisid < 0)) break;  // should not happen
        if (!jt.board->ValidRead()) break;
    }
    this->Valid = (i == ActuatorList.size());
    return this->Valid;
}

void mtsRobotIO1394::RobotInternal::GetData(void)
{
    PowerStatus = true;
    SafetyRelay = true;
    unsigned int singleEncoderPos;

    for (size_t index = 0; index < ActuatorList.size(); index++) {
        AmpIO *board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
        if (!board || (axis < 0)) continue;
        singleEncoderPos = board->GetEncoderPosition(axis);
        encPosRaw[index] = ((int)(singleEncoderPos << 8)) >> 8; // convert from 24 bits signed stored in 32 unsigned to 32 signed
        encVelRaw[index] = board->GetEncoderVelocity(axis);
        analogInRaw[index] = board->GetAnalogInput(axis);
        // digitalIn[index] = board->GetDigitalInput(axis);
        motorFeedbackCurrentRaw[index] = board->GetMotorCurrent(axis);
        ampEnable[index] = board->GetAmpEnable(axis);
        ampStatus[index] = board->GetAmpStatus(axis);
        PowerStatus &= board->GetPowerStatus();
        SafetyRelay &= board->GetSafetyRelayStatus();
    }
}

void mtsRobotIO1394::RobotInternal::ConvertRawToSI(void)
{
    EncoderRawToSI(encPosRaw, this->PositionActuatorGet.Position());
    EncoderRawToDeltaPosSI(encVelRaw, encVel);
    AnalogInBitsToVolts(analogInRaw, analogInVolts);
    DriveBitsToFeedbackAmps(motorFeedbackCurrentRaw, motorFeedbackCurrent);

    if (this->HasActuatorToJointCoupling) {
        this->PositionJoint.ProductOf(this->ActuatorToJointPosition,
                                      this->PositionActuatorGet.Position());
    } else {
        this->PositionJoint.Assign(this->PositionActuatorGet.Position());
    }
    this->PositionJointGet.Position().Assign(this->PositionJoint);
}

//************************ PROTECTED METHODS ******************************

void mtsRobotIO1394::RobotInternal::GetNumberOfActuators(int & placeHolder) const
{
    placeHolder = ActuatorList.size();
}

void mtsRobotIO1394::RobotInternal::GetNumberOfJoints(int & placeHolder) const
{
    placeHolder = this->NumberOfJoints;
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

void mtsRobotIO1394::RobotInternal::SetWatchdogPeriod(const unsigned long & period_ms)
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

void mtsRobotIO1394::RobotInternal::SetAmpEnable(const vctBoolVec & ampControl)
{
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        AmpIO *board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
        if (!board || (axis < 0)) continue;
        board->SetAmpEnable(axis, ampControl[index]);
    }
}

void mtsRobotIO1394::RobotInternal::SetTorqueJoint(const prmForceTorqueJointSet & jointTorques)
{
    // todo - put temporary variables as data members in class to avoid dynamic new/delete
    vctDoubleVec actuatorTorques(ActuatorList.size());
    actuatorTorques.ProductOf(this->JointToActuatorTorque, jointTorques.ForceTorque());
    vctDoubleVec actuatorAmps(ActuatorList.size());
    this->DriveNmToAmps(actuatorTorques, actuatorAmps);
    this->SetMotorCurrent(actuatorAmps);
}

void mtsRobotIO1394::RobotInternal::SetMotorCurrentRaw(const vctLongVec & mcur)
{
    if (mcur.size() != motorControlCurrentRaw.size()) {
        CMN_LOG_CLASS_RUN_ERROR << robotName << "::SetMotorCurrentRaw: size mismatch ("
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

void mtsRobotIO1394::RobotInternal::SetMotorCurrent(const vctDoubleVec & mcur)
{
    if (mcur.size() != motorControlCurrent.size()) {
        CMN_LOG_CLASS_RUN_ERROR << robotName << "::SetMotorCurrent: size mismatch ("
                                << mcur.size() << ", " << motorControlCurrent.size() << ")" << std::endl;
        return;
    }
    motorControlCurrent = mcur;
    //MotorCurrentToDAC(motorControlCurrent, motorControlCurrentRaw);
    DriveAmpsToBits(motorControlCurrent, motorControlCurrentRaw);
    SetMotorCurrentRaw(motorControlCurrentRaw);
}

void mtsRobotIO1394::RobotInternal::PreloadEncoders(void)
{
    vctIntVec encoderValues(ActuatorList.size());
    for (size_t index = 0; index < ActuatorList.size();index++) {
        // encoderValues[index] = ActuatorList[index].encoder.BitsPreload;
    }
    this->SetEncoderPositionRaw(encoderValues);
}

void mtsRobotIO1394::RobotInternal::ResetSingleEncoder(const int & index)
{
    mtsRobotIO1394::RobotInternal::ActuatorInfo::Encoder * encoder = &(ActuatorList[index].encoder);
    encoder->BitsToPosSIOffset = -(encPosRaw[index] * encoder->BitsToPosSIScale);
}

void mtsRobotIO1394::RobotInternal::SetEncoderPositionRaw(const vctIntVec & epos)
{
    if (epos.size() != encSetPosRaw.size()) {
        CMN_LOG_CLASS_RUN_ERROR << robotName << "::SetEncoderPositionRaw: size mismatch ("
                                << epos.size() << ", " << encSetPosRaw.size() << ")" << std::endl;
        return;
    }
    encSetPosRaw = epos;
    for (size_t index = 0; index < ActuatorList.size(); index++){
        AmpIO *board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
        if (!board || (axis < 0)) continue;
        board->WriteEncoderPreload(axis, encSetPosRaw[index]);
    }
}

void mtsRobotIO1394::RobotInternal::SetEncoderPosition(const vctDoubleVec & epos)
{
    if (epos.size() != encSetPos.size()) {
        CMN_LOG_CLASS_RUN_ERROR << robotName << "::SetEncoderPosition: size mismatch ("
                                << epos.size() << ", " << encSetPos.size() << ")" << std::endl;
        return;
    }
    encSetPos = epos;
    EncoderSIToRaw(encSetPos, encSetPosRaw);
    std::cerr << "=========================================== fix me " << CMN_LOG_DETAILS << std::endl;
}

// Unit Conversions
void mtsRobotIO1394::RobotInternal::EncoderRawToSI(const vctIntVec & fromData, vctDoubleVec & toData) const
{
    const mtsRobotIO1394::RobotInternal::ActuatorInfo::Encoder * encoder;
    toData.SetAll(0.0);
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        encoder = &(ActuatorList[index].encoder);
        toData[index] = fromData[index] * encoder->BitsToPosSIScale + encoder->BitsToPosSIOffset;
        // toData[index] = (fromData[index] * bitsToPosSIScale) + bitsToPosSIOffset;
        // toData[index] = (fromData[index] - bitsToPosSIOffset) * bitsToPosSIScale;
    }
    encoder = &(ActuatorList[0].encoder);
}

void mtsRobotIO1394::RobotInternal::EncoderSIToRaw(const vctDoubleVec & fromData, vctIntVec &toData) const
{
    toData.SetAll(0L);
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        double bitsToPosSIScale = ActuatorList[index].encoder.BitsToPosSIScale;
        double bitsToPosSIOffset = ActuatorList[index].encoder.BitsToPosSIOffset;
        toData[index] = (fromData[index]-bitsToPosSIOffset) / bitsToPosSIScale;
    }
}

void mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosSI(const vctLongVec & fromData, vctDoubleVec & toData) const
{
    toData.SetAll(0.0);
    for (size_t index = 0; index < ActuatorList.size();index++) {
        double bitsToDeltaPosSIScale = ActuatorList[index].encoder.BitsToDeltaPosSIScale;
        double bitsToDeltaPosSIOffset = ActuatorList[index].encoder.BitsToDeltaPosSIOffset;
        toData[index] = (fromData[index] * bitsToDeltaPosSIScale) + bitsToDeltaPosSIOffset;
    }
}

void mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosT(const vctLongVec & fromData, vctDoubleVec & toData) const
{
    toData.SetAll(0.0);
    for (size_t index = 0; index < ActuatorList.size();index++) {
        double bitsToDeltaTScale = ActuatorList[index].encoder.BitsToDeltaTScale;
        double bitsToDeltaTOffset = ActuatorList[index].encoder.BitsToDeltaTOffset;
        toData[index] = (fromData[index] * bitsToDeltaTScale) + bitsToDeltaTOffset ;
    }
}

void mtsRobotIO1394::RobotInternal::DriveAmpsToBits(const vctDoubleVec & fromData, vctLongVec & toData) const
{
    toData.SetAll(0L);
    for (size_t index = 0; index < ActuatorList.size();index++) {
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

void mtsRobotIO1394::RobotInternal::DriveBitsToFeedbackAmps(const vctLongVec & fromData, vctDoubleVec & toData) const
{
    toData.SetAll(0.0);
    for (size_t index = 0; index < ActuatorList.size();index++){
        double bitsToFeedbackAmpsScale = ActuatorList[index].drive.BitsToFeedbackAmpsScale;
        double bitsToFeedbackAmpsOffset = ActuatorList[index].drive.BitsToFeedbackAmpsOffset;
        toData[index] = (fromData[index] * bitsToFeedbackAmpsScale) + bitsToFeedbackAmpsOffset;
    }
}

void mtsRobotIO1394::RobotInternal::DriveNmToAmps(const vctDoubleVec & fromData, vctDoubleVec & toData) const
{
    toData.SetAll(0L);
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        double nmToAmps = ActuatorList[index].drive.NmToAmpsScale;
        toData[index] = fromData[index]*nmToAmps;
    }
}

void mtsRobotIO1394::RobotInternal::DriveAmpsToNm(const vctDoubleVec & fromData, vctDoubleVec & toData) const
{
    toData.SetAll(0.0);
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        double nmToAmps = ActuatorList[index].drive.NmToAmpsScale;
        toData[index] = fromData[index] / nmToAmps;
    }
}

void mtsRobotIO1394::RobotInternal::AnalogInBitsToVolts(const vctLongVec & fromData, vctDoubleVec & toData) const
{
    toData.SetAll(0.0);
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        double bitsToVoltsScale = ActuatorList[index].analogIn.BitsToVoltsScale;
        double bitsToVoltsOffset = ActuatorList[index].analogIn.BitsToVoltsOffset;
        toData[index] = (fromData[index] * bitsToVoltsScale) + bitsToVoltsOffset;
    }
}

void mtsRobotIO1394::RobotInternal::AnalogInVoltsToPosSI(const vctDoubleVec & fromData, vctDoubleVec & toData) const
{
    toData.SetAll(0.0);
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        double voltsToPosSIScale = ActuatorList[index].analogIn.VoltsToPosSIScale;
        double voltsToPosSIOffset = ActuatorList[index].analogIn.VoltsToPosSIOffset;

        toData[index] = (fromData[index]  * voltsToPosSIScale) + voltsToPosSIOffset;
    }
}

//Future Works:
//Variable config file with ENC/POT. Set flags so Get/Set is properly configured. Configure Function should account for different
//Layouts.
