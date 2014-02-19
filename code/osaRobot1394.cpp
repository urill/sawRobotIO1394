/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen, Peter Kazanzides, Jonathan Bohren
  Created on: 2011-06-10

  (C) Copyright 2011-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cmath>

#include <sawRobotIO1394/osaRobot1394.h>

#ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <cisstCommon/cmnUnits.h>
#include <cisstOSAbstraction/osaSleep.h>
#endif

#include <AmpIO.h>

using namespace sawRobotIO1394;

osaRobot1394::osaRobot1394(const osaRobot1394Configuration & config,
                           const size_t max_consecutive_current_safety_violations):
    // IO Structures
    mActuatorInfo(),
    mUniqueBoards(),
    // State Initialization
    mValid(false),
    mPowerStatus(false),
    mPreviousPowerStatus(false),
    mWatchdogStatus(false),
    mPreviousWatchdogStatus(false),
    mSafetyRelay(false),
    mCurrentSafetyViolationsCounter(0),
    mCurrentSafetyViolationsMaximum(max_consecutive_current_safety_violations)
{
    this->Configure(config);
}

void osaRobot1394::Configure(const osaRobot1394Configuration & config)
{
    // Store the configuration
    mConfiguration = config;

    //  info
    mName = config.Name;
    mNumberOfActuators = config.NumberOfActuators;
    mNumberOfJoints = config.NumberOfJoints;
    mPotType = config.PotLocation;

    // Low-level API
    mActuatorInfo.resize(mNumberOfActuators);

    // Initialize state vectors to the appropriate sizes
    mActuatorPowerStatus.resize(mNumberOfActuators);
    mActuatorPowerEnabled.resize(mNumberOfActuators);
    mDigitalInputs.resize(mNumberOfActuators);
    mPotBits.resize(mNumberOfActuators);
    mEncoderPositionBits.resize(mNumberOfActuators);
    mEncoderVelocityBits.resize(mNumberOfActuators);
    mActuatorCurrentBitsCommand.resize(mNumberOfActuators);
    mActuatorCurrentBitsFeedback.resize(mNumberOfActuators);
    mTimeStamp.resize(mNumberOfActuators);
    mPotVoltage.resize(mNumberOfActuators);
    mPotPosition.resize(mNumberOfActuators);
    mEncoderPosition.resize(mNumberOfActuators);
    mEncoderPositionPrev.resize(mNumberOfActuators);
    mEncoderVelocity.resize(mNumberOfActuators);
    mJointPosition.resize(mNumberOfJoints);
    mJointVelocity.resize(mNumberOfJoints);
    mActuatorCurrentCommand.resize(mNumberOfActuators);
    mActuatorEffortCommand.resize(mNumberOfActuators);
    mActuatorCurrentFeedback.resize(mNumberOfActuators);
    mActuatorEffortFeedback.resize(mNumberOfActuators);

    // Initialize property vectors to the appropriate sizes
    mJointType.resize(mNumberOfJoints);

    mEffortToCurrentScales.resize(mNumberOfActuators);
    mCurrentToBitsScales.resize(mNumberOfActuators);
    mCurrentToBitsOffsets.resize(mNumberOfActuators);
    mBitsToCurrentScales.resize(mNumberOfActuators);
    mBitsToCurrentOffsets.resize(mNumberOfActuators);
    mActuatorEffortCommandLimits.resize(mNumberOfActuators);
    mActuatorCurrentCommandLimits.resize(mNumberOfActuators);
    mActuatorCurrentFeedbackLimits.resize(mNumberOfActuators);

    mBitsToPositionScales.resize(mNumberOfActuators);
    mBitsToPositionOffsets.resize(mNumberOfActuators);
    mBitsToDPositionScales.resize(mNumberOfActuators);
    mBitsToDPositionOffsets.resize(mNumberOfActuators);
    mBitsToDTimeScales.resize(mNumberOfActuators);
    mBitsToDTimeOffsets.resize(mNumberOfActuators);
    mBitsToVecocityScales.resize(mNumberOfActuators);
    mBitsToVelocityOffsets.resize(mNumberOfActuators);

    mBitsToVoltageScales.resize(mNumberOfActuators);
    mBitsToVoltageOffsets.resize(mNumberOfActuators);
    mVoltageToPositionScales.resize(mNumberOfActuators);
    mVoltageToPositionOffsets.resize(mNumberOfActuators);
    mCountsPerTurn.resize(mNumberOfActuators);

    mTemperature.resize(mNumberOfActuators);

    // Construct property vectors
    for (size_t i = 0; i < mNumberOfActuators; i++) {

        // Local references to the config properties
        const osaActuator1394Configuration & actuator = config.Actuators[i];
        const osaDrive1394Configuration & drive = actuator.Drive;
        const osaEncoder1394Configuration & encoder = actuator.Encoder;
        const osaPot1394Configuration & pot = actuator.Pot;

        mJointType[i] = actuator.JointType;

        mEffortToCurrentScales[i]         = drive.EffortToCurrentScale;
        mCurrentToBitsScales[i]           = drive.CurrentToBitsScale;
        mCurrentToBitsOffsets[i]          = drive.CurrentToBitsOffset;
        mBitsToCurrentScales[i]           = drive.BitsToCurrentScale;
        mBitsToCurrentOffsets[i]          = drive.BitsToCurrentOffset;
        mActuatorEffortCommandLimits[i]   = drive.ActuatorEffortCommandLimit;
        mActuatorCurrentCommandLimits[i]  = drive.ActuatorCurrentCommandLimit;
        // 120% of command curret is in the acceptable range
        // Add 50 mA for non motorized actuators due to a2d noise
        mActuatorCurrentFeedbackLimits[i] = 1.2 * mActuatorCurrentCommandLimits[i] + (50.0 / 1000.0);

        mBitsToPositionScales[i]   = encoder.BitsToPositionScale;
        mBitsToPositionOffsets[i]  = encoder.BitsToPositionOffset;
        mBitsToDPositionScales[i]  = encoder.BitsToDPositionScale;
        mBitsToDPositionOffsets[i] = encoder.BitsToDPositionOffset;
        mBitsToDTimeScales[i]      = encoder.BitsToDTimeScale;
        mBitsToDTimeOffsets[i]     = encoder.BitsToDTimeOffset;
        mBitsToVecocityScales[i]   = encoder.BitsToVelocityScale;
        mBitsToVelocityOffsets[i]  = encoder.BitsToVelocityOffset;
        mCountsPerTurn[i]          = encoder.CountsPerTurn;

        mBitsToVoltageScales[i]      = pot.BitsToVoltageScale;
        mBitsToVoltageOffsets[i]     = pot.BitsToVoltageOffset;
        mVoltageToPositionScales[i]  = pot.VoltageToPositionScale;
        mVoltageToPositionOffsets[i] = pot.VoltageToPositionOffset;

        // Initialize state vectors
        mEncoderPosition[i] = 0.0;
        mEncoderPositionPrev[i] = 0.0;
        mActuatorCurrentCommand[i] = 0.0;
        mActuatorCurrentFeedback[i] = 0.0;
    }

    // Compute effort command limits
    mJointEffortCommandLimits = mConfiguration.ActuatorToJointEffort * mActuatorEffortCommandLimits;
}

void osaRobot1394::SetBoards(const std::vector<osaActuatorMapping> & boards)
{
    if (boards.size() != mNumberOfActuators) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": number of boards different than the number of actuators."));
    }

    for (size_t i = 0; i < mNumberOfActuators; i++) {
        // Store this board
        mActuatorInfo[i].board = boards[i].board;
        mActuatorInfo[i].axis = boards[i].axis;
        // Construct a list of unique boards
        mUniqueBoards[boards[i].board->GetBoardId()] = boards[i].board;
    }
}

void osaRobot1394::PollValidity(void)
{
    // Make sure the boards have been configured
    if (mNumberOfActuators != mActuatorInfo.size()) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": number of boards different than the number of actuators."));
    }

    // Store previous state
    mPreviousPowerStatus = mPowerStatus;
    mPreviousWatchdogStatus = mWatchdogStatus;

    // Initialize flags
    mValid = true;
    mPowerStatus = true;
    mSafetyRelay = true;
    mWatchdogStatus = true;

    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        mValid &= board->second->ValidRead();
        mPowerStatus &= board->second->GetPowerStatus();
        mSafetyRelay &= board->second->GetSafetyRelayStatus();
        mWatchdogStatus &= board->second->GetWatchdogTimeoutStatus();
    }

    if (!mValid) {
        std::stringstream message;
        message << this->Name() << ": read error on board(s) ";
        for (unique_board_iterator board = mUniqueBoards.begin();
             board != mUniqueBoards.end();
             ++board) {
            if (!board->second->ValidRead()) {
                message << static_cast<int>(board->second->GetBoardId()) << " ";
            }
        }
        cmnThrow(osaRuntimeError1394(message.str()));
    }
}

void osaRobot1394::PollState(void)
{
    // Poll data
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        AmpIO * board = mActuatorInfo[i].board;
        int axis = mActuatorInfo[i].axis;

        if (!board || (axis < 0)) continue; // We probably don't need this check any more

        mTimeStamp[i] = board->GetTimestamp() * 1.0 / 49125000.0;
        mDigitalInputs[i] = board->GetDigitalInput();

        // convert from 24 bits signed stored in 32 unsigned to 32 signed
        mEncoderPositionBits[i] = ((int)(board->GetEncoderPosition(axis) << 8)) >> 8;
        // convert from 16 bits signed stored in 32 unsigned to 32 signed
        mEncoderVelocityBits[i] = ((int)(board->GetEncoderVelocity(axis) << 16)) >> 16;

        mPotBits[i] = board->GetAnalogInput(axis);

        mActuatorCurrentBitsFeedback[i] = board->GetMotorCurrent(axis);
        mActuatorPowerEnabled[i] = board->GetAmpEnable(axis);
        mActuatorPowerStatus[i] = board->GetAmpStatus(axis);

        // first temperature corresponds to first 2 actuators, second to last 2
        // board reports temperature in celsius * 2
        mTemperature[i] = (board->GetAmpTemperature(axis / 2)) / 2.0;
    }
}

void osaRobot1394::ConvertState(void)
{
    // Perform read conversions
    EncoderBitsToPosition(mEncoderPositionBits, mEncoderPosition);
    mJointPosition.ProductOf(mConfiguration.ActuatorToJointPosition, mEncoderPosition);

    // Velocity computation
    const int mode = 1;
    switch (mode) {
    case 0:
        // use vel estimation from FPGA
        EncoderBitsToVelocity(mEncoderVelocityBits, mEncoderVelocity);
        mJointVelocity.ProductOf(mConfiguration.ActuatorToJointPosition, mEncoderVelocity);
        break;
    case 1:
        // use encoder position divided by time
        mEncoderVelocity.DifferenceOf(mEncoderPosition, mEncoderPositionPrev);
        mEncoderVelocity.ElementwiseDivide(mTimeStamp);
        mJointVelocity.ProductOf(mConfiguration.ActuatorToJointPosition, mEncoderVelocity);
        break;
    default:
        abort();
        break;
    }

    ActuatorBitsToCurrent(mActuatorCurrentBitsFeedback, mActuatorCurrentFeedback);
    ActuatorCurrentToEffort(mActuatorCurrentFeedback, mActuatorEffortFeedback);

    PotBitsToVoltage(mPotBits, mPotVoltage);
    PotVoltageToPosition(mPotVoltage, mPotPosition);
}

void osaRobot1394::CheckState(void)
{
    // Save EncoderPositionPrev
    mEncoderPositionPrev.Assign(mEncoderPosition);

    // Perform safety checks
    bool current_safety_violation = false;
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        if (fabs(mActuatorCurrentFeedback[i]) >= mActuatorCurrentFeedbackLimits[i]) {
            current_safety_violation = true;
        }
    }

    if (current_safety_violation) {
        mCurrentSafetyViolationsCounter++;
    } else {
        mCurrentSafetyViolationsCounter = 0;
    }

    if (mCurrentSafetyViolationsCounter > mCurrentSafetyViolationsMaximum) {
        this->DisablePower();
        cmnThrow(osaRuntimeError1394(this->Name() + ": too many consecutive current safety violations.  Power has been disabled."));
    }

    // check safety amp disable
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        AmpIO_UInt32 safetyAmpDisable = board->second->GetSafetyAmpDisable();
        if (safetyAmpDisable) {
            cmnThrow(osaRuntimeError1394(this->Name() + ": hardware current safety ampdisable tripped." + mTimeStamp.ToString()));
        }
    }
}

void osaRobot1394::EnablePower(void)
{
    this->EnableBoardsPower();
    osaSleep(50.0 * cmn_ms);
    this->SetActuatorPower(true);
}

void osaRobot1394::EnableBoardsPower(void)
{
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WriteSafetyRelay(true);
        board->second->WritePowerEnable(true);
    }
}

void osaRobot1394::DisablePower(void)
{
    // write to boards directly
    // disable all axes
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WriteAmpEnable(0x0f, 0x00);
    }

    // disable all boards
    this->DisableBoardPower();
}

void osaRobot1394::DisableBoardPower(void)
{
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WritePowerEnable(false);
        board->second->WriteSafetyRelay(false);
    }
}

void osaRobot1394::SetSafetyRelay(const bool & enabled)
{
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->SetSafetyRelay(enabled);
    }
}

void osaRobot1394::SetWatchdogPeriod(const double & periodInSeconds)
{
    uint32_t periodCounts;
    if (periodInSeconds == 0.0) {
        // Disable watchdog
        periodCounts = 0;
    } else {
        // Use at least one tick just to make sure we don't accidentaly disable
        // the truth is that the count will be so low that watchdog will
        // continuously trigger.
        periodCounts = (periodInSeconds * 1000.0) * WATCHDOG_MS_TO_COUNT;
        periodCounts = std::max(periodCounts, static_cast<uint32_t>(1));
    }

    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WriteWatchdogPeriod(periodCounts);
    }
}

void osaRobot1394::SetActuatorPower(const bool & enabled)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        mActuatorInfo[i].board->SetAmpEnable(mActuatorInfo[i].axis, enabled);
    }
}

void osaRobot1394::SetActuatorPower(const vctBoolVec & enabled)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        mActuatorInfo[i].board->SetAmpEnable(mActuatorInfo[i].axis, enabled[i]);
    }
}

void osaRobot1394::SetEncoderPosition(const vctDoubleVec & pos)
{
    vctIntVec bits(mNumberOfActuators);
    this->EncoderPositionToBits(pos, bits);
    this->SetEncoderPositionBits(bits);
}

void osaRobot1394::SetEncoderPositionBits(const vctIntVec & bits)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        mActuatorInfo[i].board->WriteEncoderPreload(mActuatorInfo[i].axis, bits[i]);
    }
}

void osaRobot1394::SetSingleEncoderPosition(const int index, const double pos)
{
    SetSingleEncoderPositionBits(index, (pos - mBitsToPositionOffsets[index]) / mBitsToPositionScales[index]);
}

void osaRobot1394::SetSingleEncoderPositionBits(const int index, const int bits)
{
    mActuatorInfo[index].board->WriteEncoderPreload(mActuatorInfo[index].axis, bits);
}

void osaRobot1394::ClipActuatorEffort(vctDoubleVec & efforts)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        efforts[i] = std::min(efforts[i], mActuatorEffortCommandLimits[i]);
        efforts[i] = std::max(efforts[i], -mActuatorEffortCommandLimits[i]);
    }
}

void osaRobot1394::ClipActuatorCurrent(vctDoubleVec & currents)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        currents[i] = std::min(currents[i], mActuatorCurrentCommandLimits[i]);
        currents[i] = std::max(currents[i], -mActuatorCurrentCommandLimits[i]);
    }
}

void osaRobot1394::SetJointEffort(const vctDoubleVec & efforts)
{
    vctDoubleVec actuator_efforts(mNumberOfActuators);
    actuator_efforts = mConfiguration.JointToActuatorEffort * efforts;
    this->SetActuatorEffort(actuator_efforts);
}

void osaRobot1394::SetActuatorEffort(const vctDoubleVec & efforts)
{
    // Convert efforts to bits and set the command
    vctDoubleVec clipped_efforts = efforts;
    vctDoubleVec currents(mNumberOfActuators);

    // this->clip_actuator_efforts(clipped_efforts);

    this->ActuatorEffortToCurrent(clipped_efforts, currents);
    this->SetActuatorCurrent(currents);
}

void osaRobot1394::SetActuatorCurrent(const vctDoubleVec & currents)
{
    // Convert amps to bits and set the command
    vctDoubleVec clipped_amps = currents;
    vctIntVec bits(mNumberOfActuators);

    this->ClipActuatorCurrent(clipped_amps);
    this->ActuatorCurrentToBits(clipped_amps, bits);
    this->SetActuatorCurrentBits(bits);

    // Store commanded amps
    mActuatorCurrentCommand = clipped_amps;
}

void osaRobot1394::SetActuatorCurrentBits(const vctIntVec & bits)
{
    for (size_t i=0; i<mNumberOfActuators; i++) {
        mActuatorInfo[i].board->SetMotorCurrent(mActuatorInfo[i].axis, bits[i]);
    }

    // Store commanded bits
    mActuatorCurrentBitsCommand = bits;
}

void osaRobot1394::CalibrateEncoderOffsetsFromPots(void)
{
    vctDoubleVec joint_positions(mNumberOfJoints);
    vctDoubleVec joint_error(mNumberOfJoints);
    vctDoubleVec actuator_error(mNumberOfActuators);

    switch(mPotType) {

    case POTENTIOMETER_UNDEFINED:
        // TODO: Return error of some kind?
        // CMN_LOG_CLASS_INIT_ERROR << "ResetEncoderOffsetUsingPotPosSI: can't set encoder offset, potentiometer's position undefined";
        break;

    case POTENTIOMETER_ON_JOINTS:
        joint_positions.ProductOf(mConfiguration.ActuatorToJointPosition, mEncoderPosition);
        joint_error.DifferenceOf(joint_positions, mPotPosition);
        actuator_error.ProductOf(mConfiguration.JointToActuatorPosition, joint_error);
        mBitsToPositionOffsets.DifferenceOf(mBitsToPositionOffsets, actuator_error);
        break;

    case POTENTIOMETER_ON_ACTUATORS:
        actuator_error.DifferenceOf(mEncoderPosition, mPotPosition);
        mBitsToPositionOffsets.DifferenceOf(mBitsToPositionOffsets, actuator_error);
        break;
    };
}

bool osaRobot1394::Valid(void) const {
    return mValid;
}

bool osaRobot1394::PowerStatus(void) const {
    return mPowerStatus;
}

bool osaRobot1394::SafetyRelay(void) const {
    return mSafetyRelay;
}

bool osaRobot1394::WatchdogStatus(void) const {
    return mWatchdogStatus;
}

const vctBoolVec & osaRobot1394::ActuatorPowerStatus(void) const {
    return mActuatorPowerStatus;
}

const vctDoubleVec & osaRobot1394::ActuatorCurrentFeedback(void) const {
    return mActuatorCurrentFeedback;
}

const vctDoubleVec & osaRobot1394::PotPosition(void) const {
    return mPotPosition;
}

const vctDoubleVec & osaRobot1394::TimeStamp(void) const {
    return mTimeStamp;
}

const vctDoubleVec & osaRobot1394::EncoderPosition(void) const {
    return mEncoderPosition;
}

const vctDoubleVec & osaRobot1394::EncoderVelocity(void) const {
    return mEncoderVelocity;
}

osaRobot1394Configuration osaRobot1394::GetConfiguration(void) const {
    return mConfiguration;
}

std::string osaRobot1394::Name(void) const {
    return mName;
}

double osaRobot1394::NumberOfJoints(void) const {
    return mNumberOfJoints;
}

double osaRobot1394::NumberOfActuators(void) const {
    return mNumberOfActuators;
}

void osaRobot1394::GetJointTypes(prmJointTypeVec & joint_types) const
{
    joint_types.resize(mNumberOfJoints);
    for (size_t i = 0; i < mNumberOfJoints; i++) {
        joint_types[i] = mJointType[i];
    }
}
void osaRobot1394::GetJointEffortCommandLimits(vctDoubleVec & limits) const
{
    limits = mJointEffortCommandLimits;
}

void osaRobot1394::GetActuatorEffortCommandLimits(vctDoubleVec & limits) const
{
    limits = mActuatorEffortCommandLimits;
}

void osaRobot1394::GetActuatorCurrentCommandLimits(vctDoubleVec & limits) const
{
    limits = mActuatorCurrentCommandLimits;
}

void osaRobot1394::EncoderPositionToBits(const vctDoubleVec & pos, vctIntVec & bits) const {
    for (size_t i = 0; i < bits.size() && i < pos.size(); i++) {
        bits[i] = static_cast<long>((pos[i] - mBitsToPositionOffsets[i]) / mBitsToPositionScales[i]);
    }
}

void osaRobot1394::EncoderBitsToPosition(const vctIntVec & bits, vctDoubleVec & pos) const {
    for (size_t i = 0; i < bits.size() && i < pos.size(); i++) {
        pos[i] = static_cast<double>(bits[i]) * mBitsToPositionScales[i] + mBitsToPositionOffsets[i];
    }
}

void osaRobot1394::EncoderBitsToDPosition(const vctIntVec & bits, vctDoubleVec & dpos) const {
    for (size_t i = 0; i < bits.size() && i < dpos.size(); i++) {
        dpos[i] = static_cast<double>(bits[i]) * mBitsToDPositionScales[i] + mBitsToDPositionOffsets[i];
    }
}

void osaRobot1394::EncoderBitsToDTime(const vctIntVec & bits, vctDoubleVec & dt) const {
    for (size_t i = 0; i < bits.size() && i < dt.size(); i++) {
        dt[i] = static_cast<double>(bits[i]) * mBitsToDTimeScales[i] + mBitsToDTimeOffsets[i];
    }
}

void osaRobot1394::EncoderBitsToVelocity(const vctIntVec & bits, vctDoubleVec & vel) const
{
    // NOTE: BitsToVecocityScales, BitsToVelocityOffsets = 0
    for (size_t i = 0; i < bits.size() && i < vel.size(); i++) {
        vel[i] = mBitsToDPositionScales[i] / static_cast<double>(bits[i]);
        if ((vel[i] < 0.001) && vel[i] > -0.001) {
            vel[i] = 0.0;
        }
    }
}

void osaRobot1394::ActuatorEffortToCurrent(const vctDoubleVec & efforts, vctDoubleVec & currents) const {
    currents.ElementwiseProductOf(efforts, mEffortToCurrentScales);
}

void osaRobot1394::ActuatorCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const {
    for (size_t i = 0; i < bits.size() && i < currents.size(); i++) {
        bits[i] = static_cast<long>(currents[i] * mCurrentToBitsScales[i] + mCurrentToBitsOffsets[i]);
    }
}

void osaRobot1394::ActuatorBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const {
    for (size_t i = 0; i < bits.size() && i < currents.size(); i++) {
        currents[i] = static_cast<double>(bits[i]) * mBitsToCurrentScales[i] + mBitsToCurrentOffsets[i];
    }
}

void osaRobot1394::ActuatorCurrentToEffort(const vctDoubleVec & currents, vctDoubleVec & efforts) const {
    efforts.ElementwiseProductOf(currents, mEffortToCurrentScales);
}

void osaRobot1394::PotBitsToVoltage(const vctIntVec & bits, vctDoubleVec & voltages) const {
    for (size_t i = 0; i < bits.size() && i < voltages.size(); i++) {
        voltages[i] = static_cast<double>(bits[i]) * mBitsToVoltageScales[i] + mBitsToVoltageOffsets[i];
    }
}

void osaRobot1394::PotVoltageToPosition(const vctDoubleVec & voltages, vctDoubleVec & pos) const {
    pos.ElementwiseProductOf(voltages, mVoltageToPositionScales);
    pos.SumOf(pos, mVoltageToPositionOffsets);
}
