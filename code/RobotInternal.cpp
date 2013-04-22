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

mtsRobotIO1394::RobotInternal::RobotInternal(const std::string & name,
                                             const mtsTaskPeriodic & owner,
                                             size_t numActuators, size_t numJoints) :
    OwnerServices(owner.Services()),
    robotName(name), ActuatorList(numActuators),
    NumberOfActuators(numActuators), NumberOfJoints(numJoints),
    TaskPeriod(owner.GetPeriodicity()),
    HasActuatorToJointCoupling(false), Valid(false),
    ampStatus(numActuators, false), ampEnable(numActuators, false),
    encPosRaw(numActuators), PositionJoint(numJoints),
    PositionJointGet(numJoints), PositionActuatorGet(numActuators),
    encVelRaw(numActuators), encVel(numActuators),
    analogInRaw(numActuators), analogInVolts(numActuators), analogInPosSI(numActuators),
    motorFeedbackCurrentRaw(numActuators), motorFeedbackCurrent(numActuators),
    TorqueJoint(numJoints),
    jointTorque(numJoints), jointTorqueMax(numJoints),
    motorControlCurrentRaw(numActuators), motorControlCurrent(numActuators),
    motorControlTorque(numActuators),
    encSetPosRaw(numActuators), encSetPos(numActuators),
    allOn(numActuators, true), allOff(numActuators, false)
{
}

mtsRobotIO1394::RobotInternal::~RobotInternal()
{
}

void mtsRobotIO1394::RobotInternal::Configure (cmnXMLPath  & xmlConfigFile, int robotNumber, AmpIO **BoardList)
{
    char path[64];
    std::string context = "Config";

    for (int i = 0; i < NumberOfActuators; i++) {
        int xmlIndex = i + 1;

        int tmpBoardID = -1;
        sprintf(path,"Robot[%d]/Actuator[%d]/@BoardID", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, tmpBoardID);
        if ((tmpBoardID < 0) || (tmpBoardID >= mtsRobotIO1394::MAX_BOARDS)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid board number " << tmpBoardID
                                     << " for actuator " << i << std::endl;
            continue;
        }
        int tmpAxisID = -1;
        sprintf(path,"Robot[%d]/Actuator[%d]/@AxisID", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, tmpAxisID);
        if ((tmpAxisID < 0) || (tmpAxisID >= 4)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid axis number " << tmpAxisID
                                     << " for actuator " << i << std::endl;
            continue;
        }
        if(BoardList[tmpBoardID] == 0) {
            BoardList[tmpBoardID] = new AmpIO(tmpBoardID);
            // Following does not properly handle the case where a board is split
            // between two robots, but there is no good way to handle that since
            // the power/safety control is per board.
            OwnBoards.push_back(BoardList[tmpBoardID]);
        }
        ActuatorList[i] = ActuatorInfo(BoardList[tmpBoardID], tmpAxisID);

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
        ActuatorList[i].analogIn.VoltsToPosSIScale *= cmnPI_180; // -------------------------------------------- adeguet1, make sure these are degrees

        sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/VoltsToPosSI/@Offset", robotNumber, xmlIndex);
        xmlConfigFile.GetXMLValue(context.c_str(), path, ActuatorList[i].analogIn. VoltsToPosSIOffset);
        ActuatorList[i].analogIn. VoltsToPosSIOffset *= cmnPI_180; // -------------------------------------------- adeguet1, make sure these are degrees
    }

    ConfigureCoupling(xmlConfigFile, robotNumber);
    UpdateJointTorqueMax();
}

void
mtsRobotIO1394::RobotInternal::ConfigureCoupling(cmnXMLPath & xmlConfigFile, int robotNumber) {
    char path[64];
    std::string context = "Config";

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
        ActuatorToJointPosition.SetSize(NumberOfJoints, NumberOfActuators, 0.0);
        JointToActuatorPosition.SetSize(NumberOfActuators, NumberOfJoints, 0.0);
        ActuatorToJointTorque.SetSize(NumberOfJoints, NumberOfActuators, 0.0);
        JointToActuatorTorque.SetSize(NumberOfActuators, NumberOfJoints, 0.0);

        ConfigureCouplingMatrix(xmlConfigFile, robotNumber, "ActuatorToJointPosition",
                                NumberOfJoints, NumberOfActuators, ActuatorToJointPosition);
        ConfigureCouplingMatrix(xmlConfigFile, robotNumber, "JointToActuatorPosition",
                                NumberOfActuators, NumberOfJoints, JointToActuatorPosition);
        ConfigureCouplingMatrix(xmlConfigFile, robotNumber, "ActuatorToJointTorque",
                                NumberOfJoints, NumberOfActuators, ActuatorToJointTorque);
        ConfigureCouplingMatrix(xmlConfigFile, robotNumber, "JointToActuatorTorque",
                                NumberOfActuators, NumberOfJoints, JointToActuatorTorque);
        //Still need to do proper alignment and such for joint/actuator situ for each matrix.
    }
    this->HasActuatorToJointCoupling = tmpCouplingAvailable;

    // make sure the coupling matrices make sense
    vctDoubleMat product, identity;
    identity.ForceAssign(vctDoubleMat::Eye(this->NumberOfActuators));
    product.SetSize(this->NumberOfActuators, this->NumberOfActuators);
    product.ProductOf(this->ActuatorToJointPosition, this->JointToActuatorPosition);
    if (!product.AlmostEqual(identity)) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureCoupling: product of position coupling matrices not identity:"
                                 << std::endl << product << std::endl;
    }
    product.ProductOf(this->ActuatorToJointTorque, this->JointToActuatorTorque);
    if (!product.AlmostEqual(identity)) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureCoupling: product of torque coupling matrices not identity:"
                                 << std::endl << product << std::endl;
    }
}

void mtsRobotIO1394::RobotInternal::ConfigureCouplingMatrix (cmnXMLPath & xmlConfigFile, int robotNumber, const char *couplingString,
                                                             int numRows, int numCols, vctDoubleMat & resultMatrix)
{
    char pathToMatrix[64];
    sprintf(pathToMatrix, "Robot[%i]/Coupling/%s", robotNumber, couplingString);

    char path[64];
    std::string context = "Config";
    std::string tmpRow = "";
    bool ssTest = false;
    int i = 0;

    for (i = 0; i < numRows; i++) {
        vctDoubleVec tmpRowVec;
        tmpRowVec.SetSize(numCols);
        std::stringstream tmpStringStream;
        tmpRow = "";
        ssTest = false;
        char tmpPath[32];
        sprintf(tmpPath, "/Row[%i]/@Val", i + 1);
        strcpy(path, pathToMatrix);
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

void mtsRobotIO1394::RobotInternal::UpdateJointTorqueMax(void)
{
    vctDoubleVec motorTorqueMax(ActuatorList.size());
    for (size_t i = 0; i < ActuatorList.size(); i++)
        motorTorqueMax[i] = ActuatorList[i].drive.MaxCurrentValue;
    DriveAmpsToNm(motorTorqueMax, motorTorqueMax);
    if (HasActuatorToJointCoupling)
        jointTorqueMax.ProductOf(ActuatorToJointTorque, motorTorqueMax);
    else
        jointTorqueMax.Assign(motorTorqueMax);
    // Make sure maximum joint torques are all positive
    jointTorqueMax.AbsSelf();
    CMN_LOG_CLASS_INIT_WARNING << "Maximum joint torques = " << jointTorqueMax << std::endl;
}

void mtsRobotIO1394::RobotInternal::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(Valid, robotName + "Valid");
    stateTable.AddData(PowerStatus, robotName + "PowerStatus");
    stateTable.AddData(SafetyRelay, robotName + "SafetyRelay");
    stateTable.AddData(WatchdogTimeout, robotName + "WatchdogTimeout");
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
    stateTable.AddData(motorFeedbackCurrentRaw, robotName + "MotorFeedbackCurrentRaw");
    stateTable.AddData(motorFeedbackCurrent, robotName + "MotorFeedbackCurrent");
    stateTable.AddData(motorControlCurrentRaw, robotName + "MotorControlCurrentRaw");
    stateTable.AddData(motorControlCurrent, robotName + "MotorControlCurrent");
}

void mtsRobotIO1394::RobotInternal::SetupInterfaces(mtsInterfaceProvided * robotInterface,
                                                    mtsInterfaceProvided * actuatorInterface,
                                                    mtsStateTable & stateTable)
{
    robotInterface->AddCommandRead(&mtsRobotIO1394::RobotInternal::GetNumberOfActuators, this,
                                   "GetNumberOfActuators");
    robotInterface->AddCommandRead(&mtsRobotIO1394::RobotInternal::GetNumberOfJoints, this,
                                   "GetNumberOfJoints");
    robotInterface->AddCommandReadState(stateTable, this->Valid, "IsValid");

    // Enable // Disable
    robotInterface->AddCommandVoid(&mtsRobotIO1394::RobotInternal::EnablePower, this, "EnablePower");
    robotInterface->AddCommandVoid(&mtsRobotIO1394::RobotInternal::DisablePower, this, "DisablePower");
    robotInterface->AddCommandVoid(&mtsRobotIO1394::RobotInternal::EnableSafetyRelay, this, "EnableSafetyRelay");
    robotInterface->AddCommandVoid(&mtsRobotIO1394::RobotInternal::DisableSafetyRelay, this, "DisableSafetyRelay");

    robotInterface->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetWatchdogPeriod, this, "SetWatchdogPeriod");

    robotInterface->AddCommandReadState(stateTable, stateTable.PeriodStats, "GetPeriodStatistics"); // mtsIntervalStatistics
    robotInterface->AddCommandReadState(stateTable, this->PowerStatus, "GetPowerStatus"); // bool
    robotInterface->AddCommandReadState(stateTable, this->SafetyRelay, "GetSafetyRelay"); // unsigned short
    robotInterface->AddCommandReadState(stateTable, this->WatchdogTimeout, "GetWatchdogTimeout"); // bool

    robotInterface->AddCommandReadState(stateTable, this->encPosRaw, "GetPositionEncoderRaw"); // vector[int]
    robotInterface->AddCommandReadState(stateTable, this->PositionJoint, "GetPosition"); // vector[double]

    robotInterface->AddCommandReadState(stateTable, this->PositionJointGet, "GetPositionJoint"); // prmPositionJointGet

    robotInterface->AddCommandReadState(stateTable, this->encVelRaw, "GetVelocityRaw");
    robotInterface->AddCommandReadState(stateTable, this->encVel, "GetVelocity");

    robotInterface->AddCommandReadState(stateTable, this->analogInRaw, "GetAnalogInputRaw");
    robotInterface->AddCommandReadState(stateTable, this->analogInVolts, "GetAnalogInputVolts");
    robotInterface->AddCommandReadState(stateTable, this->analogInPosSI, "GetAnalogInputPosSI");

    robotInterface->AddCommandReadState(stateTable, this->motorFeedbackCurrentRaw, "GetMotorFeedbackCurrentRaw");
    robotInterface->AddCommandReadState(stateTable, this->motorFeedbackCurrent, "GetMotorFeedbackCurrent");

    robotInterface->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetTorqueJoint, this, "SetTorqueJoint", TorqueJoint);
    robotInterface->AddCommandRead(&mtsRobotIO1394::RobotInternal::GetTorqueJointMax, this, "GetTorqueJointMax",
                                   jointTorqueMax);

    robotInterface->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetMotorCurrentRaw, this, "SetMotorCurrentRaw",
                                    motorControlCurrentRaw);
    robotInterface->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetMotorCurrent, this, "SetMotorCurrent",
                                    motorControlCurrent);

    robotInterface->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetEncoderPositionRaw, this, "SetEncoderPositionRaw",
                                    encSetPosRaw);
    robotInterface->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetEncoderPosition, this, "SetEncoderPosition",
                                    encSetPos);

    // unit conversion methods (Qualified Read)
    robotInterface->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::EncoderRawToSI, this,
                                            "EncoderRawToSI", encPosRaw, vctDoubleVec());
    robotInterface->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::EncoderSIToRaw, this,
                                            "EncoderSIToRaw", vctDoubleVec(), encPosRaw);
    robotInterface->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosSI, this,
                                            "EncoderRawToDeltaPosSI", encVelRaw, encVel);
    robotInterface->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::EncoderRawToDeltaPosT, this,
                                            "EncoderRawToDeltaPosT", encVelRaw, encVel);
    robotInterface->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::DriveAmpsToNm, this,
                                            "DriveAmpsToNm", motorControlCurrent, motorControlTorque);
    robotInterface->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::DriveNmToAmps, this,
                                            "DriveNmToAmps", motorControlTorque, motorControlCurrent);
    robotInterface->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::AnalogInBitsToVolts, this,
                                            "AnalogInBitsToVolts", analogInRaw, analogInVolts);

    //Debug to run Cursor Example
    robotInterface->AddCommandReadState(stateTable, this->ampEnable, "GetAmpEnable"); // vector[bool]
    robotInterface->AddCommandReadState(stateTable, this->ampStatus, "GetAmpStatus"); // vector[bool]
    robotInterface->AddCommandReadState(stateTable, this->PositionActuatorGet, "GetPositionActuator"); // prmPositionJointGet

    robotInterface->AddCommandWrite(&mtsRobotIO1394::RobotInternal::RequestAmpsToBitsOffsetUsingFeedbackAmps, this, "BiasCurrent",
                                    mtsInt(100));
    robotInterface->AddCommandVoid(&mtsRobotIO1394::RobotInternal::ResetEncoderOffsetUsingPotPosSI, this, "BiasEncoder");

    // fine tune power, board vs. axis
    actuatorInterface->AddCommandVoid(&mtsRobotIO1394::RobotInternal::EnableBoardsPower, this, "EnableBoardsPower");
    actuatorInterface->AddCommandVoid(&mtsRobotIO1394::RobotInternal::DisableBoardsPower, this, "DisableBoardsPower");
    actuatorInterface->AddCommandWrite(&mtsRobotIO1394::RobotInternal::SetAmpEnable, this, "SetAmpEnable",
                                       this->ampEnable); // vector[bool]
    actuatorInterface->AddCommandWrite(&mtsRobotIO1394::RobotInternal::ResetSingleEncoder, this, "ResetSingleEncoder"); // int
    actuatorInterface->AddCommandReadState(stateTable, this->ampEnable, "GetAmpEnable"); // vector[bool]
    actuatorInterface->AddCommandReadState(stateTable, this->ampStatus, "GetAmpStatus"); // vector[bool]

    actuatorInterface->AddCommandReadState(stateTable, this->PositionActuatorGet, "GetPositionActuator"); // prmPositionJointGet

    actuatorInterface->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::DriveAmpsToBits, this,
                                               "DriveAmpsToBits", motorFeedbackCurrent, motorFeedbackCurrentRaw);

    actuatorInterface->AddCommandQualifiedRead(&mtsRobotIO1394::RobotInternal::AnalogInVoltsToPosSI, this,
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
    WatchdogTimeout = true;
    unsigned int singleEncoderPos;
    unsigned int singleEncoderVel;

    for (size_t index = 0; index < ActuatorList.size(); index++) {
        AmpIO *board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
        if (!board || (axis < 0)) continue;
        singleEncoderPos = board->GetEncoderPosition(axis);
        encPosRaw[index] = ((int)(singleEncoderPos << 8)) >> 8; // convert from 24 bits signed stored in 32 unsigned to 32 signed
        singleEncoderVel = board->GetEncoderVelocity(axis);
        encVelRaw[index] = ((int)(singleEncoderVel << 16)) >> 16; // convert from 16 bits signed stored in 32 unsigedn to 32 signed
        analogInRaw[index] = board->GetAnalogInput(axis);
        motorFeedbackCurrentRaw[index] = board->GetMotorCurrent(axis);     
        ampEnable[index] = board->GetAmpEnable(axis);
        ampStatus[index] = board->GetAmpStatus(axis);
        PowerStatus &= board->GetPowerStatus();
        SafetyRelay &= board->GetSafetyRelayStatus();
        WatchdogTimeout &= board->GetWatchdogTimeoutStatus();
    }
}

void mtsRobotIO1394::RobotInternal::ConvertRawToSI(void)
{
    EncoderRawToSI(encPosRaw, this->PositionActuatorGet.Position());
    EncoderRawToDeltaPosSI(encVelRaw, encVel);
    AnalogInBitsToVolts(analogInRaw, analogInVolts);
    AnalogInVoltsToPosSI(analogInVolts, analogInPosSI);
    DriveBitsToFeedbackAmps(motorFeedbackCurrentRaw, motorFeedbackCurrent);

    if (this->HasActuatorToJointCoupling) {
        this->PositionJoint.ProductOf(this->ActuatorToJointPosition,
                                      this->PositionActuatorGet.Position());
    } else {
        this->PositionJoint.Assign(this->PositionActuatorGet.Position());
    }
    this->PositionJointGet.Position().Assign(this->PositionJoint);

    if (AmpsToBitsOffsetUsingFeedbackAmps.Count != 0) {
        const size_t index = --AmpsToBitsOffsetUsingFeedbackAmps.Count;
        // last motor current in amps is in motorFeedbackCurrent
        // last request current in amps is in motorControlCurrent
        AmpsToBitsOffsetUsingFeedbackAmps.ControlCurrents[index].ForceAssign(motorControlCurrent);
        AmpsToBitsOffsetUsingFeedbackAmps.FeedbackCurrents[index].ForceAssign(motorFeedbackCurrent);
        if (index == 0) {
            this->ResetAmpsToBitsOffsetUsingFeedbackAmps();
        }
    }
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

#include <cisstOSAbstraction/osaSleep.h>

void mtsRobotIO1394::RobotInternal::EnablePower(void)
{
    // Enable boards first
    EnableBoardsPower();
    // Now, enable all amplifiers
    SetAmpEnable(allOn);
}

void mtsRobotIO1394::RobotInternal::DisablePower(void)
{
    // Disable boards first
    DisableBoardsPower();
    // Now, disable all amplifiers
    SetAmpEnable(allOff);
}

void mtsRobotIO1394::RobotInternal::EnableBoardsPower(void)
{
    // Make sure all boards are enabled.
    // Notice we use Write to board to make sure this is not buffered.
    for (size_t index = 0; index < OwnBoards.size(); index++) {
        OwnBoards[index]->WriteSafetyRelay(true);
        OwnBoards[index]->WritePowerEnable(true);
    }
    // Make sure that watchdog period is set (for now, 5*TaskPeriod).
    // We do this in EnablePower to be sure it is set (e.g., if a board
    // is connected when this component is already running).
    // PK TEMP: disabled watchdog because it would otherwise trip due to
    // the subsequent 100 msec sleep.
    // SetWatchdogPeriod(5.0*TaskPeriod);
    osaSleep(100.0 * cmn_ms); // Without the sleep, we can get power jumps and joints without power enabled
}

void mtsRobotIO1394::RobotInternal::DisableBoardsPower(void)
{
    // Make sure all boards are disabled.
    // Notice we use Write to board to make sure this is not buffered.
    for (size_t index = 0; index < OwnBoards.size(); index++) {
        OwnBoards[index]->WritePowerEnable(false);
        OwnBoards[index]->WriteSafetyRelay(false);
    }
}

void mtsRobotIO1394::RobotInternal::EnableSafetyRelay(void)
{
    for (size_t index = 0; index < OwnBoards.size(); index++)
        OwnBoards[index]->SetSafetyRelay(true);
}

void mtsRobotIO1394::RobotInternal::DisableSafetyRelay(void)
{
    for (size_t index = 0; index < OwnBoards.size(); index++)
        OwnBoards[index]->SetSafetyRelay(false);
}

void mtsRobotIO1394::RobotInternal::SetWatchdogPeriod(const double & period_sec)
{
    // write timeout period, converted to counts
    unsigned int period_count;

    // first case disable watchdog
    if (period_sec == 0){
        period_count = 0;
    } else {
        // enable watchdog
        period_count = static_cast<unsigned int> (cmnInternalTo_ms(period_sec) * WD_MSTOCOUNT);
        // if less that resolution, use at least one tick just to make sure we don't accidentaly disable
        // the truth is that the count will be so low that watchdog will continuously trigger
        if (period_count == 0) {
            period_count = 1;
        }
    }

    // write to board
    for (size_t index = 0; index < OwnBoards.size(); index++)
        OwnBoards[index]->WriteWatchdogPeriod(period_count);
}

void mtsRobotIO1394::RobotInternal::SetAmpEnable(const vctBoolVec & ampControl)
{
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        AmpIO * board = ActuatorList[index].board;
        int axis = ActuatorList[index].axisid;
        if (!board || (axis < 0)) continue;
        board->SetAmpEnable(axis, ampControl[index]);
    }
}

void mtsRobotIO1394::RobotInternal::SetTorqueJoint(const prmForceTorqueJointSet & jointTorques)
{
    TorqueJoint = jointTorques;
    jointTorque.Assign(TorqueJoint.ForceTorque());
    // Check joint torque limits
    jointTorque.ElementwiseClipIn(jointTorqueMax);
    motorControlTorque.ProductOf(JointToActuatorTorque, jointTorque);
    DriveNmToAmps(motorControlTorque, motorControlCurrent);
    SetMotorCurrent(motorControlCurrent);
}

void mtsRobotIO1394::RobotInternal::GetTorqueJointMax(vctDoubleVec &maxTorques) const
{
    if (maxTorques.size() != jointTorqueMax.size()) {
        CMN_LOG_CLASS_RUN_ERROR << robotName << "::GetTorqueJointMax: size mismatch ("
                                << maxTorques.size() << ", " << jointTorqueMax.size() << ")" << std::endl;
        return;
    }
    maxTorques.Assign(jointTorqueMax);
}

void mtsRobotIO1394::RobotInternal::SetMotorCurrentRaw(const vctLongVec & mcur)
{
    if (mcur.size() != motorControlCurrentRaw.size()) {
        CMN_LOG_CLASS_RUN_ERROR << robotName << "::SetMotorCurrentRaw: size mismatch ("
                                << mcur.size() << ", " << motorControlCurrentRaw.size() << ")" << std::endl;
        return;
    }
    motorControlCurrentRaw = mcur;
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


void mtsRobotIO1394::RobotInternal::RequestAmpsToBitsOffsetUsingFeedbackAmps(const mtsInt & numberOfSamples)
{
    AmpsToBitsOffsetUsingFeedbackAmps.Count = numberOfSamples;
    AmpsToBitsOffsetUsingFeedbackAmps.ControlCurrents.SetSize(numberOfSamples);
    AmpsToBitsOffsetUsingFeedbackAmps.FeedbackCurrents.SetSize(numberOfSamples);
}

void mtsRobotIO1394::RobotInternal::ResetAmpsToBitsOffsetUsingFeedbackAmps(void)
{
    vctDoubleVec currentAverage(NumberOfActuators);
    vctDoubleVec feedbackAverage(NumberOfActuators);
    const size_t numberOfSamples = AmpsToBitsOffsetUsingFeedbackAmps.ControlCurrents.size();
    for (size_t index = 0;
         index < numberOfSamples;
         ++index) {
        currentAverage.Add(AmpsToBitsOffsetUsingFeedbackAmps.ControlCurrents[index]);
        feedbackAverage.Add(AmpsToBitsOffsetUsingFeedbackAmps.FeedbackCurrents[index]);
    }
    currentAverage.Divide(numberOfSamples);
    feedbackAverage.Divide(numberOfSamples);
    vctDoubleVec errorCurrent(NumberOfActuators);
    errorCurrent.DifferenceOf(currentAverage, feedbackAverage);
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        ActuatorList[index].drive.AmpsToBitsOffset += (static_cast<double>(errorCurrent[index]) * ActuatorList[index].drive.AmpsToBitsScale);
    }
    // apply the new offsets
    DriveAmpsToBits(this->motorControlCurrent, this->motorControlCurrentRaw);
    SetMotorCurrentRaw(motorControlCurrentRaw);
}

void mtsRobotIO1394::RobotInternal::ResetEncoderOffsetUsingPotPosSI(void)
{
    vctDoubleVec posErrorJoint(NumberOfJoints);
    posErrorJoint.DifferenceOf(this->PositionJointGet.Position(), this->analogInPosSI);
    vctDoubleVec posErrorActuator(NumberOfActuators);
    posErrorActuator.ProductOf(this->JointToActuatorPosition, posErrorJoint);
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        ActuatorList[index].encoder.BitsToPosSIOffset -= posErrorActuator[index];
    }
}

void mtsRobotIO1394::RobotInternal::ResetSingleEncoder(const int & index)
{
    AmpIO *board = ActuatorList[index].board;
    int axis = ActuatorList[index].axisid;
    if (!board || (axis < 0)) {
        return;
    }
    board->WriteEncoderPreload(axis, 0);
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
    SetEncoderPositionRaw(encSetPosRaw);
}

// Unit Conversions
void mtsRobotIO1394::RobotInternal::EncoderRawToSI(const vctIntVec & fromData, vctDoubleVec & toData) const
{
    const mtsRobotIO1394::RobotInternal::ActuatorInfo::Encoder * encoder;
    toData.SetAll(0.0);
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        encoder = &(ActuatorList[index].encoder);
        toData[index] = fromData[index] * encoder->BitsToPosSIScale + encoder->BitsToPosSIOffset;
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
    /**
     * See configGenerator.py for how to compute bitsToDeltaPosSIScale
     * Velocoity = bitsToDeltaPosSIScale / timeCounter
     */

    toData.SetAll(0.0);
    for (size_t index = 0; index < ActuatorList.size();index++) {
        double bitsToDeltaPosSIScale = ActuatorList[index].encoder.BitsToDeltaPosSIScale;
        double bitsToDeltaPosSIOffset = ActuatorList[index].encoder.BitsToDeltaPosSIOffset;
        toData[index] = bitsToDeltaPosSIScale / (fromData[index]);
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
    for (size_t index = 0; index < ActuatorList.size(); index++) {
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

// fromData and toData can be same vector
void mtsRobotIO1394::RobotInternal::DriveNmToAmps(const vctDoubleVec & fromData, vctDoubleVec & toData) const
{
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        double nmToAmps = ActuatorList[index].drive.NmToAmpsScale;
        toData[index] = fromData[index]*nmToAmps;
    }
}

// fromData and toData can be same vector
void mtsRobotIO1394::RobotInternal::DriveAmpsToNm(const vctDoubleVec & fromData, vctDoubleVec & toData) const
{
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

// fromData and toData can be same vector
void mtsRobotIO1394::RobotInternal::AnalogInVoltsToPosSI(const vctDoubleVec & fromData, vctDoubleVec & toData) const
{
    for (size_t index = 0; index < ActuatorList.size(); index++) {
        double voltsToPosSIScale = ActuatorList[index].analogIn.VoltsToPosSIScale;
        double voltsToPosSIOffset = ActuatorList[index].analogIn.VoltsToPosSIOffset;

        toData[index] = (fromData[index]  * voltsToPosSIScale) + voltsToPosSIOffset;
    }
}

//Future Works:
//Variable config file with ENC/POT. Set flags so Get/Set is properly configured. Configure Function should account for different
//Layouts.
