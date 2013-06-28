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

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>

#include "mtsIO1394Robot.h"

using namespace sawRobotIO1394;

mtsIO1394Robot::mtsIO1394Robot(const cmnGenericObject & owner,
                               const osaIO1394::RobotConfiguration & config):
    osaIO1394Robot(config, 100, 1000),
    OwnerServices(owner.Services())
{
}

void mtsIO1394Robot::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(Valid_, Name_ + "Valid");
    stateTable.AddData(PowerStatus_, Name_ + "PowerStatus");
    stateTable.AddData(SafetyRelay_, Name_ + "SafetyRelay");
    stateTable.AddData(WatchdogStatus_, Name_ + "WatchdogTimeout");
    stateTable.AddData(BoardTemperature_, Name_ + "AmpTemperature");
    stateTable.AddData(ActuatorPowerStatus_, Name_ + "AmpStatus");
    stateTable.AddData(ActuatorPowerEnabled_, Name_ + "AmpEnable");
    stateTable.AddData(EncoderPositionBits_, Name_ + "PosRaw");
    stateTable.AddData(JointPosition_, Name_ + "PositionJoint");
    stateTable.AddData(EncoderVelocityBits_, Name_ + "VelRaw");
    stateTable.AddData(EncoderVelocity_, Name_ + "vel");
    stateTable.AddData(PotBits_, Name_ + "AnalogInRaw");
    stateTable.AddData(PotVoltage_, Name_ + "AnalogInVolts");
    stateTable.AddData(PotPosition_, Name_ + "AnalogInPosSI");
    stateTable.AddData(ActuatorCurrentBitsCommand_, Name_ + "MotorControlCurrentRaw");
    stateTable.AddData(ActuatorCurrentCommand_, Name_ + "MotorControlCurrent");
    stateTable.AddData(ActuatorCurrentBitsFeedback_, Name_ + "MotorFeedbackCurrentRaw");
    stateTable.AddData(ActuatorCurrentFeedback_, Name_ + "MotorFeedbackCurrent");

    stateTable.AddData(PositionJointGet, Name_ + "PositionJointGet");
    stateTable.AddData(PositionActuatorGet, Name_ + "PositionActuatorGet");
}

void mtsIO1394Robot::GetNumberOfActuators(int & num_actuators) const {
    num_actuators = this->NumberOfActuators();
}

void mtsIO1394Robot::GetNumberOfJoints(int & num_joints) const {
    num_joints = this->NumberOfJoints();
}

void mtsIO1394Robot::SetTorqueJoint(const prmForceTorqueJointSet & efforts) {
    this->SetJointEffort(efforts.ForceTorque());
}

void mtsIO1394Robot::EnableSafetyRelay(void) {
    this->SetSafetyRelay(true);
}

void mtsIO1394Robot::DisableSafetyRelay(void) {
    this->SetSafetyRelay(false);
}

void mtsIO1394Robot::ResetSingleEncoder(const int & index) {
    this->SetSingleEncoderPosition(index, 0.0);
}

void mtsIO1394Robot::SetupInterfaces(mtsInterfaceProvided * robotInterface,
                                     mtsInterfaceProvided * actuatorInterface,
                                     mtsStateTable & stateTable)
{
    osaIO1394Robot * thisBase = dynamic_cast<osaIO1394Robot *>(this);
    CMN_ASSERT(thisBase);

    robotInterface->AddCommandRead(&mtsIO1394Robot::GetNumberOfActuators, this,
                                   "GetNumberOfActuators");
    robotInterface->AddCommandRead(&mtsIO1394Robot::GetNumberOfJoints, this,
                                   "GetNumberOfJoints");
    robotInterface->AddCommandReadState(stateTable, this->Valid_,
                                        "IsValid");

    // Enable // Disable
    robotInterface->AddCommandVoid(&osaIO1394Robot::EnablePower, thisBase,
                                   "EnablePower");
    robotInterface->AddCommandVoid(&osaIO1394Robot::DisablePower, thisBase,
                                   "DisablePower");
    robotInterface->AddCommandVoid(&mtsIO1394Robot::EnableSafetyRelay, this,
                                   "EnableSafetyRelay");
    robotInterface->AddCommandVoid(&mtsIO1394Robot::DisableSafetyRelay, this,
                                   "DisableSafetyRelay");

    robotInterface->AddCommandWrite(&osaIO1394Robot::SetWatchdogPeriod, thisBase,
                                    "SetWatchdogPeriod");

    robotInterface->AddCommandReadState(stateTable, stateTable.PeriodStats,
                                        "GetPeriodStatistics"); // mtsIntervalStatistics
    robotInterface->AddCommandReadState(stateTable, PowerStatus_,
                                        "GetPowerStatus"); // bool
    robotInterface->AddCommandReadState(stateTable, SafetyRelay_,
                                        "GetSafetyRelay"); // unsigned short
    robotInterface->AddCommandReadState(stateTable, WatchdogStatus_,
                                        "GetWatchdogTimeout"); // bool
    robotInterface->AddCommandReadState(stateTable, BoardTemperature_,
                                        "GetAmpTemperature"); // vector[double]

    robotInterface->AddCommandReadState(stateTable, EncoderPositionBits_,
                                        "GetPositionEncoderRaw"); // vector[int]
    robotInterface->AddCommandReadState(stateTable, JointPosition_,
                                        "GetPosition"); // vector[double]

    robotInterface->AddCommandReadState(stateTable, this->PositionJointGet,
                                        "GetPositionJoint"); // prmPositionJointGet

    robotInterface->AddCommandReadState(stateTable, EncoderVelocityBits_,
                                        "GetVelocityRaw");
    robotInterface->AddCommandReadState(stateTable, EncoderVelocity_,
                                        "GetVelocity");

    robotInterface->AddCommandReadState(stateTable, PotBits_,
                                        "GetAnalogInputRaw");
    robotInterface->AddCommandReadState(stateTable, PotVoltage_,
                                        "GetAnalogInputVolts");
    robotInterface->AddCommandReadState(stateTable, PotPosition_,
                                        "GetAnalogInputPosSI");

    robotInterface->AddCommandReadState(stateTable, ActuatorCurrentBitsFeedback_,
                                        "GetMotorFeedbackCurrentRaw");
    robotInterface->AddCommandReadState(stateTable, ActuatorCurrentFeedback_,
                                        "GetMotorFeedbackCurrent");

    robotInterface->AddCommandWrite(&mtsIO1394Robot::SetTorqueJoint, this,
                                    "SetTorqueJoint", TorqueJoint);
    robotInterface->AddCommandRead(&osaIO1394Robot::GetJointEffortCommandLimits, thisBase,
                                   "GetTorqueJointMax", JointEffortCommandLimits_);

    robotInterface->AddCommandWrite(&osaIO1394Robot::SetActuatorCurrentBits, thisBase,
                                    "SetMotorCurrentRaw", ActuatorCurrentBitsCommand_);
    robotInterface->AddCommandWrite(&osaIO1394Robot::SetActuatorCurrent, thisBase,
                                    "SetMotorCurrent", ActuatorCurrentCommand_);
    robotInterface->AddCommandRead(&osaIO1394Robot::GetActuatorCurrentCommandLimits, thisBase,
                                   "GetMotorCurrentMax", ActuatorCurrentCommandLimits_);
    robotInterface->AddCommandRead(&osaIO1394Robot::GetJointTypes, thisBase,
                                   "GetJointType", JointType_);

    robotInterface->AddCommandWrite(&osaIO1394Robot::SetEncoderPositionBits, thisBase,
                                    "SetEncoderPositionRaw");
    robotInterface->AddCommandWrite(&osaIO1394Robot::SetEncoderPosition, thisBase,
                                    "SetEncoderPosition");

    // unit conversion methods (Qualified Read)
    robotInterface->AddCommandQualifiedRead(&osaIO1394Robot::EncoderBitsToPosition, thisBase,
                                            "EncoderRawToSI", vctIntVec(), vctDoubleVec());
    robotInterface->AddCommandQualifiedRead(&osaIO1394Robot::EncoderPositionToBits, thisBase,
                                            "EncoderSIToRaw", vctDoubleVec(), vctIntVec());
    robotInterface->AddCommandQualifiedRead(&osaIO1394Robot::EncoderBitsToDPosition, thisBase,
                                            "EncoderRawToDeltaPosSI", EncoderVelocityBits_, EncoderVelocity_);
    robotInterface->AddCommandQualifiedRead(&osaIO1394Robot::EncoderBitsToDTime, thisBase,
                                            "EncoderRawToDeltaPosT", EncoderDTimeBits_, EncoderDTime_);
    robotInterface->AddCommandQualifiedRead(&osaIO1394Robot::ActuatorCurrentToEffort, thisBase,
                                            "DriveAmpsToNm", ActuatorCurrentCommand_, ActuatorEffortCommand_);
    robotInterface->AddCommandQualifiedRead(&osaIO1394Robot::ActuatorEffortToCurrent, thisBase,
                                            "DriveNmToAmps", ActuatorEffortCommand_, ActuatorCurrentCommand_);
    robotInterface->AddCommandQualifiedRead(&osaIO1394Robot::PotBitsToVoltage, thisBase,
                                            "AnalogInBitsToVolts", PotBits_, PotVoltage_);

    //Debug to run Cursor Example
    robotInterface->AddCommandReadState(stateTable, ActuatorPowerEnabled_,
                                        "GetAmpEnable"); // vector[bool]
    robotInterface->AddCommandReadState(stateTable, ActuatorPowerStatus_,
                                        "GetAmpStatus"); // vector[bool]
    robotInterface->AddCommandReadState(stateTable, PositionActuatorGet,
                                        "GetPositionActuator"); // prmPositionJointGet

    robotInterface->AddCommandWrite(&osaIO1394Robot::CalibrateCurrentCommandOffsetsRequest,
                                    thisBase, "BiasCurrent");
    robotInterface->AddCommandVoid(&osaIO1394Robot::CalibrateEncoderOffsetsFromPots,
                                   thisBase, "BiasEncoder");

    // Events
    robotInterface->AddEventWrite(EventTriggers.PowerStatus, "PowerStatus", false);

    // fine tune power, board vs. axis
    actuatorInterface->AddCommandVoid(&osaIO1394Robot::EnableBoardsPower, thisBase,
                                      "EnableBoardsPower");
    actuatorInterface->AddCommandVoid(&osaIO1394Robot::DisableBoardPower, thisBase,
                                      "DisableBoardsPower");
    actuatorInterface->AddCommandWrite(&osaIO1394Robot::SetActuatorPower, this,
                                       "SetAmpEnable", ActuatorPowerEnabled_); // vector[bool]
    actuatorInterface->AddCommandWrite(&mtsIO1394Robot::ResetSingleEncoder, this,
                                       "ResetSingleEncoder"); // int

    actuatorInterface->AddCommandReadState(stateTable, ActuatorPowerEnabled_,
                                           "GetAmpEnable"); // vector[bool]
    actuatorInterface->AddCommandReadState(stateTable, ActuatorPowerStatus_,
                                           "GetAmpStatus"); // vector[bool]
    actuatorInterface->AddCommandReadState(stateTable, this->PositionActuatorGet,
                                           "GetPositionActuator"); // prmPositionJointGet

    actuatorInterface->AddCommandQualifiedRead(&osaIO1394Robot::ActuatorCurrentToBits, thisBase,
                                               "DriveAmpsToBits", ActuatorCurrentFeedback_, ActuatorCurrentBitsFeedback_);
    actuatorInterface->AddCommandQualifiedRead(&osaIO1394Robot::PotVoltageToPosition, thisBase,
                                               "AnalogInVoltsToPosSI", PotVoltage_, PotPosition_);
}

void mtsIO1394Robot::TriggerEvents(void)
{
    PositionJointGet.Position().ForceAssign(JointPosition_);
    PositionActuatorGet.Position().ForceAssign(EncoderPosition_);

    if (PreviousPowerStatus_ != PowerStatus_) {
        EventTriggers.PowerStatus(PowerStatus_);
    }
}
