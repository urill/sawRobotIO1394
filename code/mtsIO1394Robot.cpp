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
    OwnerServices(owner.Services()),
    StateTableRead_(0),
    StateTableWrite_(0)
{
}

bool mtsIO1394Robot::SetupStateTables(const size_t stateTableSize,
                                      mtsStateTable * & stateTableRead,
                                      mtsStateTable * & stateTableWrite)
{
    if (StateTableRead_ || StateTableWrite_) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupStateTables: state tables have already been created" << std::endl;
        return false;
    }

    StateTableRead_ = new mtsStateTable(stateTableSize, this->Name() + "Read");
    StateTableRead_->SetAutomaticAdvance(false);
    StateTableWrite_ = new mtsStateTable(stateTableSize, this->Name() + "Write");
    StateTableWrite_->SetAutomaticAdvance(false);

    StateTableRead_->AddData(Valid_, "Valid");
    StateTableRead_->AddData(PowerStatus_, "PowerStatus");
    StateTableRead_->AddData(SafetyRelay_, "SafetyRelay");
    StateTableRead_->AddData(WatchdogStatus_, "WatchdogTimeout");
    StateTableRead_->AddData(Temperature_, "AmpTemperature");
    StateTableRead_->AddData(ActuatorPowerStatus_, "AmpStatus");
    StateTableRead_->AddData(ActuatorPowerEnabled_, "AmpEnable");
    StateTableRead_->AddData(EncoderPositionBits_, "PosRaw");
    StateTableRead_->AddData(JointPosition_, "PositionJoint");
    StateTableRead_->AddData(EncoderVelocityBits_, "VelRaw");
    StateTableRead_->AddData(EncoderVelocity_, "Vel");
    StateTableRead_->AddData(PotBits_, "AnalogInRaw");
    StateTableRead_->AddData(PotVoltage_, "AnalogInVolts");
    StateTableRead_->AddData(PotPosition_, "AnalogInPosSI");
    StateTableRead_->AddData(ActuatorCurrentBitsCommand_, "MotorControlCurrentRaw");
    StateTableRead_->AddData(ActuatorCurrentCommand_, "MotorControlCurrent");
    StateTableRead_->AddData(ActuatorCurrentBitsFeedback_, "MotorFeedbackCurrentRaw");
    StateTableRead_->AddData(ActuatorCurrentFeedback_, "MotorFeedbackCurrent");

    StateTableRead_->AddData(PositionJointGet_, "PositionJointGet");
    StateTableRead_->AddData(PositionActuatorGet_, "PositionActuatorGet");

    stateTableRead = StateTableRead_;
    stateTableWrite = StateTableWrite_;
    return true;
}

void mtsIO1394Robot::StartReadStateTable(void) {
    StateTableRead_->Start();
}


void mtsIO1394Robot::AdvanceReadStateTable(void) {
    StateTableRead_->Advance();
}

void mtsIO1394Robot::GetNumberOfActuators(int & numberOfActuators) const {
    numberOfActuators = this->NumberOfActuators();
}

void mtsIO1394Robot::GetNumberOfJoints(int & numberOfJoints) const {
    numberOfJoints = this->NumberOfJoints();
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
                                     mtsInterfaceProvided * actuatorInterface)
{
    osaIO1394Robot * thisBase = dynamic_cast<osaIO1394Robot *>(this);
    CMN_ASSERT(thisBase);

    robotInterface->AddCommandRead(&mtsIO1394Robot::GetNumberOfActuators, this,
                                   "GetNumberOfActuators");
    robotInterface->AddCommandRead(&mtsIO1394Robot::GetNumberOfJoints, this,
                                   "GetNumberOfJoints");
    robotInterface->AddCommandReadState(*StateTableRead_, this->Valid_,
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

    robotInterface->AddCommandReadState(*StateTableRead_, StateTableRead_->PeriodStats,
                                        "GetPeriodStatistics"); // mtsIntervalStatistics
    robotInterface->AddCommandReadState(*StateTableRead_, PowerStatus_,
                                        "GetPowerStatus"); // bool
    robotInterface->AddCommandReadState(*StateTableRead_, SafetyRelay_,
                                        "GetSafetyRelay"); // unsigned short
    robotInterface->AddCommandReadState(*StateTableRead_, WatchdogStatus_,
                                        "GetWatchdogTimeout"); // bool
    robotInterface->AddCommandReadState(*StateTableRead_, Temperature_,
                                        "GetAmpTemperature"); // vector[double]

    robotInterface->AddCommandReadState(*StateTableRead_, EncoderPositionBits_,
                                        "GetPositionEncoderRaw"); // vector[int]
    robotInterface->AddCommandReadState(*StateTableRead_, JointPosition_,
                                        "GetPosition"); // vector[double]

    robotInterface->AddCommandReadState(*StateTableRead_, this->PositionJointGet_,
                                        "GetPositionJoint"); // prmPositionJointGet

    robotInterface->AddCommandReadState(*StateTableRead_, EncoderVelocityBits_,
                                        "GetVelocityRaw");
    robotInterface->AddCommandReadState(*StateTableRead_, EncoderVelocity_,
                                        "GetVelocity");

    robotInterface->AddCommandReadState(*StateTableRead_, PotBits_,
                                        "GetAnalogInputRaw");
    robotInterface->AddCommandReadState(*StateTableRead_, PotVoltage_,
                                        "GetAnalogInputVolts");
    robotInterface->AddCommandReadState(*StateTableRead_, PotPosition_,
                                        "GetAnalogInputPosSI");

    robotInterface->AddCommandReadState(*StateTableRead_, ActuatorCurrentBitsFeedback_,
                                        "GetMotorFeedbackCurrentRaw");
    robotInterface->AddCommandReadState(*StateTableRead_, ActuatorCurrentFeedback_,
                                        "GetMotorFeedbackCurrent");

    robotInterface->AddCommandWrite(&mtsIO1394Robot::SetTorqueJoint, this,
                                    "SetTorqueJoint", TorqueJoint_);
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
    robotInterface->AddCommandReadState(*StateTableRead_, ActuatorPowerEnabled_,
                                        "GetAmpEnable"); // vector[bool]
    robotInterface->AddCommandReadState(*StateTableRead_, ActuatorPowerStatus_,
                                        "GetAmpStatus"); // vector[bool]
    robotInterface->AddCommandReadState(*StateTableRead_, PositionActuatorGet_,
                                        "GetPositionActuator"); // prmPositionJointGet

    robotInterface->AddCommandWrite(&osaIO1394Robot::CalibrateCurrentCommandOffsetsRequest,
                                    thisBase, "BiasCurrent");
    robotInterface->AddCommandVoid(&osaIO1394Robot::CalibrateEncoderOffsetsFromPots,
                                   thisBase, "BiasEncoder");
    robotInterface->AddCommandWrite(&mtsIO1394Robot::ResetSingleEncoder, this,
                                    "ResetSingleEncoder"); // int

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

    actuatorInterface->AddCommandReadState(*StateTableRead_, ActuatorPowerEnabled_,
                                           "GetAmpEnable"); // vector[bool]
    actuatorInterface->AddCommandReadState(*StateTableRead_, ActuatorPowerStatus_,
                                           "GetAmpStatus"); // vector[bool]
    actuatorInterface->AddCommandReadState(*StateTableRead_, this->PositionActuatorGet_,
                                           "GetPositionActuator"); // prmPositionJointGet

    actuatorInterface->AddCommandQualifiedRead(&osaIO1394Robot::ActuatorCurrentToBits, thisBase,
                                               "DriveAmpsToBits", ActuatorCurrentFeedback_, ActuatorCurrentBitsFeedback_);
    actuatorInterface->AddCommandQualifiedRead(&osaIO1394Robot::PotVoltageToPosition, thisBase,
                                               "AnalogInVoltsToPosSI", PotVoltage_, PotPosition_);
}

void mtsIO1394Robot::CheckState(void)
{
    PositionJointGet_.Position().ForceAssign(JointPosition_);
    PositionActuatorGet_.Position().ForceAssign(EncoderPosition_);

    if (PreviousPowerStatus_ != PowerStatus_) {
        EventTriggers.PowerStatus(PowerStatus_);
    }
}
