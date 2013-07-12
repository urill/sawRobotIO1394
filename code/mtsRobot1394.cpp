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

#include "mtsRobot1394.h"

using namespace sawRobotIO1394;

mtsRobot1394::mtsRobot1394(const cmnGenericObject & owner,
                           const osaRobot1394Configuration & config):
    osaRobot1394(config, 100, 1000),
    OwnerServices(owner.Services()),
    StateTableRead_(0),
    StateTableWrite_(0)
{
}

mtsRobot1394::~mtsRobot1394()
{
    delete StateTableRead_;
    delete StateTableWrite_;
}

bool mtsRobot1394::SetupStateTables(const size_t stateTableSize,
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
    StateTableRead_->AddData(ActuatorCurrentBitsFeedback_, "MotorFeedbackCurrentRaw");
    StateTableRead_->AddData(ActuatorCurrentFeedback_, "MotorFeedbackCurrent");

    StateTableRead_->AddData(PositionJointGet_, "PositionJointGet");
    StateTableRead_->AddData(PositionActuatorGet_, "PositionActuatorGet");

    StateTableWrite_->AddData(ActuatorCurrentBitsCommand_, "MotorControlCurrentRaw");
    StateTableWrite_->AddData(ActuatorCurrentCommand_, "MotorControlCurrent");

    stateTableRead = StateTableRead_;
    stateTableWrite = StateTableWrite_;
    return true;
}

void mtsRobot1394::StartReadStateTable(void) {
    StateTableRead_->Start();
}

void mtsRobot1394::AdvanceReadStateTable(void) {
    StateTableRead_->Advance();
}

void mtsRobot1394::StartWriteStateTable(void) {
    StateTableWrite_->Start();
}

void mtsRobot1394::AdvanceWriteStateTable(void) {
    StateTableWrite_->Advance();
}

void mtsRobot1394::GetNumberOfActuators(int & numberOfActuators) const {
    numberOfActuators = this->NumberOfActuators();
}

void mtsRobot1394::GetNumberOfJoints(int & numberOfJoints) const {
    numberOfJoints = this->NumberOfJoints();
}

void mtsRobot1394::SetTorqueJoint(const prmForceTorqueJointSet & efforts) {
    this->SetJointEffort(efforts.ForceTorque());
}

void mtsRobot1394::EnableSafetyRelay(void) {
    this->SetSafetyRelay(true);
}

void mtsRobot1394::DisableSafetyRelay(void) {
    this->SetSafetyRelay(false);
}

void mtsRobot1394::ResetSingleEncoder(const int & index) {
    this->SetSingleEncoderPosition(index, 0.0);
}

void mtsRobot1394::SetupInterfaces(mtsInterfaceProvided * robotInterface,
                                   mtsInterfaceProvided * actuatorInterface)
{
    osaRobot1394 * thisBase = dynamic_cast<osaRobot1394 *>(this);
    CMN_ASSERT(thisBase);

    robotInterface->AddCommandRead(&mtsRobot1394::GetNumberOfActuators, this,
                                   "GetNumberOfActuators");
    robotInterface->AddCommandRead(&mtsRobot1394::GetNumberOfJoints, this,
                                   "GetNumberOfJoints");
    robotInterface->AddCommandReadState(*StateTableRead_, this->Valid_,
                                        "IsValid");

    // Enable // Disable
    robotInterface->AddCommandVoid(&osaRobot1394::EnablePower, thisBase,
                                   "EnablePower");
    robotInterface->AddCommandVoid(&osaRobot1394::DisablePower, thisBase,
                                   "DisablePower");
    robotInterface->AddCommandVoid(&mtsRobot1394::EnableSafetyRelay, this,
                                   "EnableSafetyRelay");
    robotInterface->AddCommandVoid(&mtsRobot1394::DisableSafetyRelay, this,
                                   "DisableSafetyRelay");

    robotInterface->AddCommandWrite(&osaRobot1394::SetWatchdogPeriod, thisBase,
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

    robotInterface->AddCommandReadState(*StateTableWrite_, ActuatorCurrentCommand_,
                                        "GetMotorRequestedCurrent");

    robotInterface->AddCommandWrite(&mtsRobot1394::SetTorqueJoint, this,
                                    "SetTorqueJoint", TorqueJoint_);
    robotInterface->AddCommandRead(&osaRobot1394::GetJointEffortCommandLimits, thisBase,
                                   "GetTorqueJointMax", JointEffortCommandLimits_);

    robotInterface->AddCommandWrite(&osaRobot1394::SetActuatorCurrentBits, thisBase,
                                    "SetMotorCurrentRaw", ActuatorCurrentBitsCommand_);
    robotInterface->AddCommandWrite(&osaRobot1394::SetActuatorCurrent, thisBase,
                                    "SetMotorCurrent", ActuatorCurrentCommand_);
    robotInterface->AddCommandRead(&osaRobot1394::GetActuatorCurrentCommandLimits, thisBase,
                                   "GetMotorCurrentMax", ActuatorCurrentCommandLimits_);
    robotInterface->AddCommandRead(&osaRobot1394::GetJointTypes, thisBase,
                                   "GetJointType", JointType_);

    robotInterface->AddCommandWrite(&osaRobot1394::SetEncoderPositionBits, thisBase,
                                    "SetEncoderPositionRaw");
    robotInterface->AddCommandWrite(&osaRobot1394::SetEncoderPosition, thisBase,
                                    "SetEncoderPosition");

    // unit conversion methods (Qualified Read)
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::EncoderBitsToPosition, thisBase,
                                            "EncoderRawToSI", vctIntVec(), vctDoubleVec());
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::EncoderPositionToBits, thisBase,
                                            "EncoderSIToRaw", vctDoubleVec(), vctIntVec());
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::EncoderBitsToDPosition, thisBase,
                                            "EncoderRawToDeltaPosSI", EncoderVelocityBits_, EncoderVelocity_);
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::EncoderBitsToDTime, thisBase,
                                            "EncoderRawToDeltaPosT", EncoderDTimeBits_, EncoderDTime_);
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::ActuatorCurrentToEffort, thisBase,
                                            "DriveAmpsToNm", ActuatorCurrentCommand_, ActuatorEffortCommand_);
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::ActuatorEffortToCurrent, thisBase,
                                            "DriveNmToAmps", ActuatorEffortCommand_, ActuatorCurrentCommand_);
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::PotBitsToVoltage, thisBase,
                                            "AnalogInBitsToVolts", PotBits_, PotVoltage_);

    //Debug to run Cursor Example
    robotInterface->AddCommandReadState(*StateTableRead_, ActuatorPowerEnabled_,
                                        "GetAmpEnable"); // vector[bool]
    robotInterface->AddCommandReadState(*StateTableRead_, ActuatorPowerStatus_,
                                        "GetAmpStatus"); // vector[bool]
    robotInterface->AddCommandReadState(*StateTableRead_, PositionActuatorGet_,
                                        "GetPositionActuator"); // prmPositionJointGet

    robotInterface->AddCommandWrite(&osaRobot1394::CalibrateCurrentCommandOffsetsRequest,
                                    thisBase, "BiasCurrent");
    robotInterface->AddCommandVoid(&osaRobot1394::CalibrateEncoderOffsetsFromPots,
                                   thisBase, "BiasEncoder");
    robotInterface->AddCommandWrite(&mtsRobot1394::ResetSingleEncoder, this,
                                    "ResetSingleEncoder"); // int

    // Events
    robotInterface->AddEventWrite(EventTriggers.PowerStatus, "PowerStatus", false);

    // fine tune power, board vs. axis
    actuatorInterface->AddCommandVoid(&osaRobot1394::EnableBoardsPower, thisBase,
                                      "EnableBoardsPower");
    actuatorInterface->AddCommandVoid(&osaRobot1394::DisableBoardPower, thisBase,
                                      "DisableBoardsPower");
    actuatorInterface->AddCommandWrite(&osaRobot1394::SetActuatorPower, this,
                                       "SetAmpEnable", ActuatorPowerEnabled_); // vector[bool]
    actuatorInterface->AddCommandWrite(&mtsRobot1394::ResetSingleEncoder, this,
                                       "ResetSingleEncoder"); // int

    actuatorInterface->AddCommandReadState(*StateTableRead_, ActuatorPowerEnabled_,
                                           "GetAmpEnable"); // vector[bool]
    actuatorInterface->AddCommandReadState(*StateTableRead_, ActuatorPowerStatus_,
                                           "GetAmpStatus"); // vector[bool]
    actuatorInterface->AddCommandReadState(*StateTableRead_, this->PositionActuatorGet_,
                                           "GetPositionActuator"); // prmPositionJointGet

    actuatorInterface->AddCommandQualifiedRead(&osaRobot1394::ActuatorCurrentToBits, thisBase,
                                               "DriveAmpsToBits", ActuatorCurrentFeedback_, ActuatorCurrentBitsFeedback_);
    actuatorInterface->AddCommandQualifiedRead(&osaRobot1394::PotVoltageToPosition, thisBase,
                                               "AnalogInVoltsToPosSI", PotVoltage_, PotPosition_);
}

void mtsRobot1394::CheckState(void)
{
    PositionJointGet_.Position().ForceAssign(JointPosition_);
    PositionActuatorGet_.Position().ForceAssign(EncoderPosition_);

    if (PreviousPowerStatus_ != PowerStatus_) {
        EventTriggers.PowerStatus(PowerStatus_);
    }
}
