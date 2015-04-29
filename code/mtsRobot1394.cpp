/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2015 Johns Hopkins University (JHU), All Rights Reserved.

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
    osaRobot1394(config, 100),
    OwnerServices(owner.Services()),
    mStateTableRead(0),
    mStateTableWrite(0),
    mFirstWatchdog(true)
{
}

mtsRobot1394::~mtsRobot1394()
{
    delete mStateTableRead;
    delete mStateTableWrite;
}

bool mtsRobot1394::SetupStateTables(const size_t stateTableSize,
                                    mtsStateTable * & stateTableRead,
                                    mtsStateTable * & stateTableWrite)
{
    if (mStateTableRead || mStateTableWrite) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupStateTables: state tables have already been created for robot: "
                                 << this->Name() << std::endl;
        return false;
    }

    mStateTableRead = new mtsStateTable(stateTableSize, this->Name() + "Read");
    mStateTableRead->SetAutomaticAdvance(false);
    mStateTableWrite = new mtsStateTable(stateTableSize, this->Name() + "Write");
    mStateTableWrite->SetAutomaticAdvance(false);

    mStateTableRead->AddData(mValid, "Valid");
    mStateTableRead->AddData(mPowerStatus, "PowerStatus");
    mStateTableRead->AddData(mSafetyRelay, "SafetyRelay");
    mStateTableRead->AddData(mWatchdogStatus, "WatchdogTimeout");
    mStateTableRead->AddData(mActuatorTemperature, "ActuatorTemperature");
    mStateTableRead->AddData(mActuatorPowerStatus, "ActuatorPowerStatus");
    mStateTableRead->AddData(mActuatorPowerEnabled, "ActuatorPowerEnabled");
    mStateTableRead->AddData(mEncoderPositionBits, "PosRaw");
    mStateTableRead->AddData(mJointPosition, "PositionJoint");
    mStateTableRead->AddData(mPotsToEncodersError, "PotsToEncoderError");
    mStateTableRead->AddData(mEncoderVelocityBits, "VelRaw");
    mStateTableRead->AddData(mEncoderVelocity, "Vel");
    mStateTableRead->AddData(mJointTorque, "Effortjoint");
    mStateTableRead->AddData(mPotBits, "AnalogInRaw");
    mStateTableRead->AddData(mPotVoltage, "AnalogInVolts");
    mStateTableRead->AddData(mPotPosition, "AnalogInPosSI");
    mStateTableRead->AddData(mActuatorCurrentBitsFeedback, "ActuatorFeedbackCurrentRaw");
    mStateTableRead->AddData(mActuatorCurrentFeedback, "ActuatorFeedbackCurrent");
    mStateTableRead->AddData(mActuatorEffortFeedback, "mActuatorEffortFeedback");

    mPositionJointGet.SetSize(mNumberOfJoints);
    mPositionJointGet.Timestamps().SetAll(0.0);
    mStateTableRead->AddData(mPositionJointGet, "PositionJointGet");

    mPositionActuatorGet.SetSize(mNumberOfActuators);
    mPositionActuatorGet.Timestamps().SetAll(0.0);
    mStateTableRead->AddData(mPositionActuatorGet, "PositionActuatorGet");

    mStateTableRead->AddData(mVelocityJointGet, "VelocityJointGet");

    mStateTableWrite->AddData(mActuatorCurrentBitsCommand, "ActuatorControlCurrentRaw");
    mStateTableWrite->AddData(mActuatorCurrentCommand, "ActuatorControlCurrent");

    mStateTableRead->AddData(mBrakePowerStatus, "BrakePowerStatus");
    mStateTableRead->AddData(mBrakePowerEnabled, "BrakePowerEnabled");
    mStateTableWrite->AddData(mBrakeCurrentBitsCommand, "BrakeControlCurrentRaw");
    mStateTableWrite->AddData(mBrakeCurrentCommand, "BrakeControlCurrent");
    mStateTableRead->AddData(mBrakeCurrentFeedback, "BrakeFeedbackCurrent");
    mStateTableRead->AddData(mBrakeTemperature, "BrakeTemperature");

    stateTableRead = mStateTableRead;
    stateTableWrite = mStateTableWrite;
    return true;
}

void mtsRobot1394::StartReadStateTable(void) {
    mStateTableRead->Start();
}

void mtsRobot1394::AdvanceReadStateTable(void) {
    mStateTableRead->Advance();
}

void mtsRobot1394::StartWriteStateTable(void) {
    mStateTableWrite->Start();
}

void mtsRobot1394::AdvanceWriteStateTable(void) {
    mStateTableWrite->Advance();
}

void mtsRobot1394::GetNumberOfActuators(int & numberOfActuators) const {
    numberOfActuators = this->NumberOfActuators();
}

void mtsRobot1394::GetNumberOfJoints(int & numberOfJoints) const {
    numberOfJoints = this->NumberOfJoints();
}

void mtsRobot1394::GetSerialNumber(int & serialNumber) const {
    serialNumber = this->SerialNumber();
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
    robotInterface->AddCommandRead(&mtsRobot1394::GetSerialNumber, this,
                                   "GetSerialNumber");
    robotInterface->AddCommandReadState(*mStateTableRead, this->mValid,
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

    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorPowerEnabled,
                                        "GetActuatorAmpEnable"); // vector[bool]
    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorPowerStatus,
                                        "GetActuatorAmpStatus"); // vector[bool]

    robotInterface->AddCommandReadState(*mStateTableRead, mStateTableRead->PeriodStats,
                                        "GetPeriodStatistics"); // mtsIntervalStatistics
    robotInterface->AddCommandReadState(*mStateTableRead, mPowerStatus,
                                        "GetPowerStatus"); // bool
    robotInterface->AddCommandReadState(*mStateTableRead, mSafetyRelay,
                                        "GetSafetyRelay"); // unsigned short
    robotInterface->AddCommandReadState(*mStateTableRead, mWatchdogStatus,
                                        "GetWatchdogTimeout"); // bool
    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorTemperature,
                                        "GetActuatorAmpTemperature"); // vector[double]

    robotInterface->AddCommandReadState(*mStateTableRead, mEncoderPositionBits,
                                        "GetPositionEncoderRaw"); // vector[int]
    robotInterface->AddCommandReadState(*mStateTableRead, mJointPosition,
                                        "GetPosition"); // vector[double]
    robotInterface->AddCommandReadState(*mStateTableRead, mJointTorque,
                                        "GetTorqueJoint"); // vector[double]
    robotInterface->AddCommandReadState(*mStateTableRead, mPotsToEncodersError,
                                        "GetPotsToEncodersError"); // vector[double]

    robotInterface->AddCommandReadState(*mStateTableRead, this->mPositionJointGet,
                                        "GetPositionJoint"); // prmPositionJointGet

    robotInterface->AddCommandReadState(*mStateTableRead, mEncoderVelocityBits,
                                        "GetVelocityRaw"); // vector[int]
    robotInterface->AddCommandReadState(*mStateTableRead, mEncoderVelocity,
                                        "GetVelocity"); // vector[double]

    robotInterface->AddCommandReadState(*mStateTableRead, this->mVelocityJointGet,
                                        "GetVelocityJoint"); // prmVelocityJointGet

    robotInterface->AddCommandReadState(*mStateTableRead, mPotBits,
                                        "GetAnalogInputRaw");
    robotInterface->AddCommandReadState(*mStateTableRead, mPotVoltage,
                                        "GetAnalogInputVolts");
    robotInterface->AddCommandReadState(*mStateTableRead, mPotPosition,
                                        "GetAnalogInputPosSI");

    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorCurrentBitsFeedback,
                                        "GetActuatorFeedbackCurrentRaw");
    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorCurrentFeedback,
                                        "GetActuatorFeedbackCurrent");
    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorEffortFeedback,
                                        "GetActuatorEffortFeedback");
    robotInterface->AddCommandReadState(*mStateTableWrite, mActuatorCurrentCommand,
                                        "GetActuatorRequestedCurrent");

    robotInterface->AddCommandWrite(&osaRobot1394::UsePotsForSafetyCheck, thisBase,
                                    "UsePotsForSafetyCheck", mUsePotsForSafetyCheck);
    robotInterface->AddCommandWrite(&osaRobot1394::SetPotsToEncodersTolerance, thisBase,
                                    "SetPotsToEncodersTolerance", mPotsToEncodersTolerance);

    robotInterface->AddCommandWrite(&osaRobot1394::SetBrakePower, this,
                                    "SetBrakeAmpEnable", mBrakePowerEnabled); // vector[bool]
    robotInterface->AddCommandReadState(*mStateTableRead, mBrakePowerEnabled,
                                        "GetBrakeAmpEnable"); // vector[bool]
    robotInterface->AddCommandReadState(*mStateTableRead, mBrakePowerStatus,
                                        "GetBrakeAmpStatus"); // vector[bool]
    robotInterface->AddCommandReadState(*mStateTableRead, mBrakeCurrentFeedback,
                                        "GetBrakeFeedbackCurrent");
    robotInterface->AddCommandReadState(*mStateTableWrite, mBrakeCurrentCommand,
                                        "GetBrakeRequestedCurrent");
    robotInterface->AddCommandReadState(*mStateTableRead, mBrakeTemperature,
                                        "GetBrakeAmpTemperature"); // vector[double]

    robotInterface->AddCommandWrite(&mtsRobot1394::SetTorqueJoint, this,
                                    "SetTorqueJoint", mTorqueJoint);
    robotInterface->AddCommandRead(&osaRobot1394::GetJointEffortCommandLimits, thisBase,
                                   "GetTorqueJointMax", mJointEffortCommandLimits);

    robotInterface->AddCommandWrite(&osaRobot1394::SetActuatorCurrentBits, thisBase,
                                    "SetActuatorCurrentRaw", mActuatorCurrentBitsCommand);
    robotInterface->AddCommandWrite(&osaRobot1394::SetActuatorCurrent, thisBase,
                                    "SetActuatorCurrent", mActuatorCurrentCommand);
    robotInterface->AddCommandRead(&osaRobot1394::GetActuatorCurrentCommandLimits, thisBase,
                                   "GetActuatorCurrentMax", mActuatorCurrentCommandLimits);
    robotInterface->AddCommandRead(&osaRobot1394::GetJointTypes, thisBase,
                                   "GetJointType", mJointType);

    robotInterface->AddCommandWrite(&osaRobot1394::SetEncoderPositionBits, thisBase,
                                    "SetEncoderPositionRaw");
    robotInterface->AddCommandWrite(&osaRobot1394::SetEncoderPosition, thisBase,
                                    "SetEncoderPosition");

    robotInterface->AddCommandVoid(&osaRobot1394::BrakeRelease, thisBase,
                                   "BrakeRelease");
    robotInterface->AddCommandVoid(&osaRobot1394::BrakeEngage, thisBase,
                                   "BrakeEngage");

    // unit conversion methods (Qualified Read)
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::EncoderBitsToPosition, thisBase,
                                            "EncoderRawToSI", vctIntVec(), vctDoubleVec());
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::EncoderPositionToBits, thisBase,
                                            "EncoderSIToRaw", vctDoubleVec(), vctIntVec());
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::EncoderBitsToDPosition, thisBase,
                                            "EncoderRawToDeltaPosSI", mEncoderVelocityBits, mEncoderVelocity);
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::EncoderBitsToDTime, thisBase,
                                            "EncoderRawToDeltaPosT", mEncoderDTimeBits, mEncoderDTime);
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::ActuatorCurrentToEffort, thisBase,
                                            "DriveAmpsToNm", mActuatorCurrentCommand, mActuatorEffortCommand);
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::ActuatorEffortToCurrent, thisBase,
                                            "DriveNmToAmps", mActuatorEffortCommand, mActuatorCurrentCommand);
    robotInterface->AddCommandQualifiedRead(&osaRobot1394::PotBitsToVoltage, thisBase,
                                            "AnalogInBitsToVolts", mPotBits, mPotVoltage);

    //
    robotInterface->AddCommandReadState(*mStateTableRead, mPositionActuatorGet,
                                        "GetPositionActuator"); // prmPositionJointGet
    robotInterface->AddCommandVoid(&osaRobot1394::CalibrateEncoderOffsetsFromPots,
                                   thisBase, "BiasEncoder");
    robotInterface->AddCommandWrite(&mtsRobot1394::ResetSingleEncoder, this,
                                    "ResetSingleEncoder"); // int

    // Events
    robotInterface->AddEventWrite(EventTriggers.PowerStatus, "PowerStatus", false);
    robotInterface->AddEventWrite(EventTriggers.WatchdogStatus, "WatchdogStatus", false);

    robotInterface->AddEventWrite(MessageEvents.Error, "Error", std::string(""));
    robotInterface->AddEventWrite(MessageEvents.Warning, "Warning", std::string(""));
    robotInterface->AddEventWrite(MessageEvents.Status, "Status", std::string(""));

    // fine tune power, board vs. axis
    actuatorInterface->AddCommandVoid(&osaRobot1394::EnableBoardsPower, thisBase,
                                      "EnableBoardsPower");
    actuatorInterface->AddCommandVoid(&osaRobot1394::DisableBoardPower, thisBase,
                                      "DisableBoardsPower");
    actuatorInterface->AddCommandWrite(&osaRobot1394::SetActuatorPower, this,
                                       "SetAmpEnable", mActuatorPowerEnabled); // vector[bool]
    actuatorInterface->AddCommandWrite(&mtsRobot1394::ResetSingleEncoder, this,
                                       "ResetSingleEncoder"); // int

    actuatorInterface->AddCommandReadState(*mStateTableRead, mActuatorPowerEnabled,
                                           "GetAmpEnable"); // vector[bool]
    actuatorInterface->AddCommandReadState(*mStateTableRead, mActuatorPowerStatus,
                                           "GetAmpStatus"); // vector[bool]
    actuatorInterface->AddCommandReadState(*mStateTableRead, this->mPositionActuatorGet,
                                           "GetPositionActuator"); // prmPositionJointGet

    actuatorInterface->AddCommandQualifiedRead(&osaRobot1394::ActuatorCurrentToBits, thisBase,
                                               "DriveAmpsToBits", mActuatorCurrentFeedback, mActuatorCurrentBitsFeedback);
    actuatorInterface->AddCommandQualifiedRead(&osaRobot1394::PotVoltageToPosition, thisBase,
                                               "AnalogInVoltsToPosSI", mPotVoltage, mPotPosition);
}

bool mtsRobot1394::CheckConfiguration(void)
{
    if ((NumberOfActuators() > 2)
        && mActuatorCurrentToBitsOffsets.Equal(mActuatorCurrentToBitsOffsets[0])) {
        CMN_LOG_CLASS_INIT_ERROR << "CheckConfiguration: all currents to bits offsets are equal, please calibrate the current offsets for arm: "
                                 << this->Name() << std::endl;
        return false;
    }
    return true;
}

void mtsRobot1394::CheckState(void)
{
    mPositionJointGet.Position().Assign(mJointPosition);
    mPositionJointGet.Timestamps().Add(mActuatorTimestamp); // todo: we don't take coupling into account here
    mPositionActuatorGet.Position().Assign(mEncoderPosition);
    mPositionActuatorGet.Timestamps().Add(mActuatorTimestamp);

    mVelocityJointGet.Velocity().ForceAssign(mJointVelocity);

    if (mPreviousPowerStatus != mPowerStatus) {
        EventTriggers.PowerStatus(mPowerStatus);
        if (!mPowerStatus) {
            MessageEvents.Error("IO: " + this->Name() + " power off");
        }
    }

    if (mPreviousWatchdogStatus != mWatchdogStatus) {
        EventTriggers.WatchdogStatus(mWatchdogStatus);
        if (!mWatchdogStatus) {
            if (mFirstWatchdog) {
                MessageEvents.Status("IO: " + this->Name() + " watchdog triggered");
                mFirstWatchdog = false;
            } else {
                MessageEvents.Error("IO: " + this->Name() + " watchdog triggered");
            }
        }
    }
}
