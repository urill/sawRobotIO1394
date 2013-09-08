/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen, Peter Kazanzides, Jonathan Bohren
  Created on: 2011-06-10

  (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights Reserved.

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
                           const size_t max_consecutive_current_safety_violations,
                           const size_t actuator_current_buffer_size):
    // IO Structures
    ActuatorInfo_(),
    UniqueBoards_(),
    // State Initialization
    Valid_(false),
    PowerStatus_(false),
    PreviousPowerStatus_(false),
    WatchdogStatus_(false),
    PreviousWatchdogStatus_(false),
    SafetyRelay_(false),
    CalibrateCurrentCommandOffsetRequested_(false),
    CurrentSafetyViolationsCounter_(0),
    CurrentSafetyViolationsMaximum_(max_consecutive_current_safety_violations),
    CalibrateCurrentBufferSize_(actuator_current_buffer_size),
    CalibrateCurrentBufferIndex_(0)
{
    this->Configure(config);
}

void osaRobot1394::Configure(const osaRobot1394Configuration & config)
{
    // Store the configuration
    Configuration_ = config;

    //  info
    Name_ = config.Name;
    NumberOfActuators_ = config.NumberOfActuators;
    NumberOfJoints_ = config.NumberOfJoints;
    PotType_ = config.PotLocation;
    CalibrateCurrentCommandBuffers_.resize(NumberOfActuators_);

    // Low-level API
    ActuatorInfo_.resize(NumberOfActuators_);

    // Initialize state vectors to the appropriate sizes
    ActuatorPowerStatus_.resize(NumberOfActuators_);
    ActuatorPowerEnabled_.resize(NumberOfActuators_);
    DigitalInputs_.resize(NumberOfActuators_);
    PotBits_.resize(NumberOfActuators_);
    EncoderPositionBits_.resize(NumberOfActuators_);
    EncoderVelocityBits_.resize(NumberOfActuators_);
    ActuatorCurrentBitsCommand_.resize(NumberOfActuators_);
    ActuatorCurrentBitsFeedback_.resize(NumberOfActuators_);
    TimeStamp_.resize(NumberOfActuators_);
    PotVoltage_.resize(NumberOfActuators_);
    PotPosition_.resize(NumberOfActuators_);
    EncoderPosition_.resize(NumberOfActuators_);
    EncoderPositionPrev_.resize(NumberOfActuators_);
    EncoderVelocity_.resize(NumberOfActuators_);
    JointPosition_.resize(NumberOfJoints_);
    JointVelocity_.resize(NumberOfJoints_);
    ActuatorCurrentCommand_.resize(NumberOfActuators_);
    ActuatorEffortCommand_.resize(NumberOfActuators_);
    ActuatorCurrentFeedback_.resize(NumberOfActuators_);
    ActuatorEffortFeedback_.resize(NumberOfActuators_);

    // Initialize property vectors to the appropriate sizes
    JointType_.resize(NumberOfJoints_);

    EffortToCurrentScales_.resize(NumberOfActuators_);
    CurrentToBitsScales_.resize(NumberOfActuators_);
    CurrentToBitsOffsets_.resize(NumberOfActuators_);
    BitsToCurrentScales_.resize(NumberOfActuators_);
    BitsToCurrentOffsets_.resize(NumberOfActuators_);
    ActuatorEffortCommandLimits_.resize(NumberOfActuators_);
    ActuatorCurrentCommandLimits_.resize(NumberOfActuators_);
    ActuatorCurrentFeedbackLimits_.resize(NumberOfActuators_);

    BitsToPositionScales_.resize(NumberOfActuators_);
    BitsToPositionOffsets_.resize(NumberOfActuators_);
    BitsToDPositionScales_.resize(NumberOfActuators_);
    BitsToDPositionOffsets_.resize(NumberOfActuators_);
    BitsToDTimeScales_.resize(NumberOfActuators_);
    BitsToDTimeOffsets_.resize(NumberOfActuators_);
    BitsToVecocityScales_.resize(NumberOfActuators_);
    BitsToVelocityOffsets_.resize(NumberOfActuators_);

    BitsToVoltageScales_.resize(NumberOfActuators_);
    BitsToVoltageOffsets_.resize(NumberOfActuators_);
    VoltageToPositionScales_.resize(NumberOfActuators_);
    VoltageToPositionOffsets_.resize(NumberOfActuators_);
    CountsPerTurn_.resize(NumberOfActuators_);

    Temperature_.resize(NumberOfActuators_);

    // Construct property vectors
    for (size_t i = 0; i < NumberOfActuators_; i++) {

        // Local references to the config properties
        const osaActuator1394Configuration & actuator = config.Actuators[i];
        const osaDrive1394Configuration & drive = actuator.Drive;
        const osaEncoder1394Configuration & encoder = actuator.Encoder;
        const osaPot1394Configuration & pot = actuator.Pot;

        JointType_[i] = actuator.JointType;

        EffortToCurrentScales_[i]         = drive.EffortToCurrentScale;
        CurrentToBitsScales_[i]            = drive.CurrentToBitsScale;
        CurrentToBitsOffsets_[i]           = drive.CurrentToBitsOffset;
        BitsToCurrentScales_[i]            = drive.BitsToCurrentScale;
        BitsToCurrentOffsets_[i]           = drive.BitsToCurrentOffset;
        ActuatorEffortCommandLimits_[i] = drive.ActuatorEffortCommandLimit;
        ActuatorCurrentCommandLimits_[i]   = drive.ActuatorCurrentCommandLimit;
        // 120% of command curret is in the acceptable range
        // Add 50 mA for non motorized actuators due to a2d noise
        ActuatorCurrentFeedbackLimits_[i]  = 1.2 * ActuatorCurrentCommandLimits_[i] + (50.0 / 1000.0);

        BitsToPositionScales_[i]   = encoder.BitsToPositionScale;
        BitsToPositionOffsets_[i]  = encoder.BitsToPositionOffset;
        BitsToDPositionScales_[i]  = encoder.BitsToDPositionScale;
        BitsToDPositionOffsets_[i] = encoder.BitsToDPositionOffset;
        BitsToDTimeScales_[i]    = encoder.BitsToDTimeScale;
        BitsToDTimeOffsets_[i]   = encoder.BitsToDTimeOffset;
        BitsToVecocityScales_[i]   = encoder.BitsToVelocityScale;
        BitsToVelocityOffsets_[i]  = encoder.BitsToVelocityOffset;
        CountsPerTurn_[i]      = encoder.CountsPerTurn;

        BitsToVoltageScales_[i]  = pot.BitsToVoltageScale;
        BitsToVoltageOffsets_[i] = pot.BitsToVoltageOffset;
        VoltageToPositionScales_[i]   = pot.VoltageToPositionScale;
        VoltageToPositionOffsets_[i]  = pot.VoltageToPositionOffset;

        // Initialize state vectors
        EncoderPosition_[i] = 0.0;
        EncoderPositionPrev_[i] = 0.0;
        ActuatorCurrentCommand_[i] = 0.0;
        ActuatorCurrentFeedback_[i] = 0.0;
    }

    // Compute effort command limits
    JointEffortCommandLimits_ = Configuration_.ActuatorToJointEffort * ActuatorEffortCommandLimits_;
}

void osaRobot1394::SetBoards(const std::vector<osaActuatorMapping> & boards)
{
    if (boards.size() != NumberOfActuators_) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": number of boards different than the number of actuators."));
    }

    for (size_t i = 0; i < NumberOfActuators_; i++) {
        // Store this board
        ActuatorInfo_[i].board = boards[i].board;
        ActuatorInfo_[i].axis = boards[i].axis;
        // Construct a list of unique boards
        UniqueBoards_[boards[i].board->GetBoardId()] = boards[i].board;
    }
}

void osaRobot1394::PollValidity(void)
{
    // Make sure the boards have been configured
    if (NumberOfActuators_ != ActuatorInfo_.size()) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": number of boards different than the number of actuators."));
    }

    // Store previous state
    PreviousPowerStatus_ = PowerStatus_;
    PreviousWatchdogStatus_ = WatchdogStatus_;

    // Initialize flags
    Valid_ = true;
    PowerStatus_ = true;
    SafetyRelay_ = true;
    WatchdogStatus_ = true;

    for (unique_board_iterator board = UniqueBoards_.begin();
         board != UniqueBoards_.end();
         ++board) {
        Valid_ &= board->second->ValidRead();
        PowerStatus_ &= board->second->GetPowerStatus();
        SafetyRelay_ &= board->second->GetSafetyRelayStatus();
        WatchdogStatus_ &= board->second->GetWatchdogTimeoutStatus();
    }

    if (!Valid_) {
        std::stringstream message;
        message << this->Name() << ": read error on board(s) ";
        for (unique_board_iterator board = UniqueBoards_.begin();
             board != UniqueBoards_.end();
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
    for (size_t i = 0; i < NumberOfActuators_; i++) {
        AmpIO * board = ActuatorInfo_[i].board;
        int axis = ActuatorInfo_[i].axis;

        if (!board || (axis < 0)) continue; // We probably don't need this check any more

        TimeStamp_[i] = board->GetTimestamp() * 1.0 / 49125000.0;
        DigitalInputs_[i] = board->GetDigitalInput();

        // convert from 24 bits signed stored in 32 unsigned to 32 signed
        EncoderPositionBits_[i] = ((int)(board->GetEncoderPosition(axis) << 8)) >> 8;
        // convert from 16 bits signed stored in 32 unsigned to 32 signed
        EncoderVelocityBits_[i] = ((int)(board->GetEncoderVelocity(axis) << 16)) >> 16;

        PotBits_[i] = board->GetAnalogInput(axis);

        ActuatorCurrentBitsFeedback_[i] = board->GetMotorCurrent(axis);
        ActuatorPowerEnabled_[i] = board->GetAmpEnable(axis);
        ActuatorPowerStatus_[i] = board->GetAmpStatus(axis);

        // first temperature corresponds to first 2 actuators, second to last 2
        // board reports temperature in celsius * 2
        Temperature_[i] = (board->GetAmpTemperature(axis / 2)) / 2.0;
    }
}

void osaRobot1394::ConvertState(void)
{
    // Perform read conversions
    EncoderBitsToPosition(EncoderPositionBits_, EncoderPosition_);
    JointPosition_ = Configuration_.ActuatorToJointPosition * EncoderPosition_;

    // Use vel estimation from FPGA
    EncoderBitsToVelocity(EncoderVelocityBits_, EncoderVelocity_);
    JointVelocity_ = Configuration_.ActuatorToJointPosition * EncoderVelocity_;

    ActuatorBitsToCurrent(ActuatorCurrentBitsFeedback_, ActuatorCurrentFeedback_);
    ActuatorCurrentToEffort(ActuatorCurrentFeedback_, ActuatorEffortFeedback_);

    PotBitsToVoltage(PotBits_, PotVoltage_);
    PotVoltageToPosition(PotVoltage_, PotPosition_);
}

void osaRobot1394::CheckState(void)
{
    // Save EncoderPositionPrev
    EncoderPositionPrev_.Assign(EncoderPosition_);

    // Store currents for biasing and re-bias if appropriate
    if (CalibrateCurrentCommandOffsetRequested_
        && (CalibrateCurrentBufferIndex_ < CalibrateCurrentBufferSize_)) {
        CalibrateCurrentCommandBuffers_[CalibrateCurrentBufferIndex_] = ActuatorCurrentCommand_;
        CalibrateCurrentFeedbackBuffers_[CalibrateCurrentBufferIndex_] = ActuatorCurrentFeedback_;
        CalibrateCurrentBufferIndex_++;

        // Only re-bias when we have collected enough samples
        if (CalibrateCurrentBufferIndex_ == CalibrateCurrentBufferSize_) {
            CalibrateCurrentCommandOffsets();
            CalibrateCurrentCommandOffsetRequested_ = false;
        }
    }

    // Perform safety checks
    bool current_safety_violation = false;
    for (size_t i = 0; i < NumberOfActuators_; i++) {
        if (fabs(ActuatorCurrentFeedback_[i]) >= ActuatorCurrentFeedbackLimits_[i]) {
            current_safety_violation = true;
        }
    }

    if (current_safety_violation) {
        CurrentSafetyViolationsCounter_++;
    } else {
        CurrentSafetyViolationsCounter_ = 0;
    }

    if (CurrentSafetyViolationsCounter_ > CurrentSafetyViolationsMaximum_) {
        this->DisablePower();
        cmnThrow(osaRuntimeError1394(this->Name() + ": too many consecutive current safety violations.  Power has been disabled."));
    }

    // check safety amp disable
    for (unique_board_iterator board = UniqueBoards_.begin();
         board != UniqueBoards_.end();
         ++board) {
        AmpIO_UInt32 safetyAmpDisable = board->second->GetSafetyAmpDisable();
        if (safetyAmpDisable) {
            cmnThrow(osaRuntimeError1394(this->Name() + ": hardware current safety ampdisable tripped." + TimeStamp_.ToString()));
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
    for (unique_board_iterator board = UniqueBoards_.begin();
         board != UniqueBoards_.end();
         ++board) {
        board->second->WriteSafetyRelay(true);
        board->second->WritePowerEnable(true);
    }
}

void osaRobot1394::DisablePower(void)
{
    // write to boards directly
    // disable all axes
    for (unique_board_iterator board = UniqueBoards_.begin();
         board != UniqueBoards_.end();
         ++board) {
        board->second->WriteAmpEnable(0x0f, 0x00);
    }

    // disable all boards
    this->DisableBoardPower();
}

void osaRobot1394::DisableBoardPower(void)
{
    for (unique_board_iterator board = UniqueBoards_.begin();
         board != UniqueBoards_.end();
         ++board) {
        board->second->WritePowerEnable(false);
        board->second->WriteSafetyRelay(false);
    }
}

void osaRobot1394::SetSafetyRelay(const bool & enabled)
{
    for (unique_board_iterator board = UniqueBoards_.begin();
         board != UniqueBoards_.end();
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

    for (unique_board_iterator board = UniqueBoards_.begin();
         board != UniqueBoards_.end();
         ++board) {
        board->second->WriteWatchdogPeriod(periodCounts);
    }
}

void osaRobot1394::SetActuatorPower(const bool & enabled)
{
    for (size_t i = 0; i < NumberOfActuators_; i++) {
        ActuatorInfo_[i].board->SetAmpEnable(ActuatorInfo_[i].axis, enabled);
    }
}

void osaRobot1394::SetActuatorPower(const vctBoolVec & enabled)
{
    for (size_t i = 0; i < NumberOfActuators_; i++) {
        ActuatorInfo_[i].board->SetAmpEnable(ActuatorInfo_[i].axis, enabled[i]);
    }
}

void osaRobot1394::SetEncoderPosition(const vctDoubleVec & pos)
{
    vctIntVec bits(NumberOfActuators_);
    this->EncoderPositionToBits(pos, bits);
    this->SetEncoderPositionBits(bits);
}

void osaRobot1394::SetEncoderPositionBits(const vctIntVec & bits)
{
    for (size_t i = 0; i < NumberOfActuators_; i++) {
        ActuatorInfo_[i].board->WriteEncoderPreload(ActuatorInfo_[i].axis, bits[i]);
    }
}

void osaRobot1394::SetSingleEncoderPosition(const int index, const double pos)
{
    SetSingleEncoderPositionBits(index, (pos - BitsToPositionOffsets_[index]) / BitsToPositionScales_[index]);
}

void osaRobot1394::SetSingleEncoderPositionBits(const int index, const int bits)
{
    ActuatorInfo_[index].board->WriteEncoderPreload(ActuatorInfo_[index].axis, bits);
}

void osaRobot1394::ClipActuatorEffort(vctDoubleVec & efforts)
{
    for (size_t i=0; i<NumberOfActuators_; i++) {
        efforts[i] = std::min(efforts[i], ActuatorEffortCommandLimits_[i]);
        efforts[i] = std::max(efforts[i], -ActuatorEffortCommandLimits_[i]);
    }
}

void osaRobot1394::ClipActuatorCurrent(vctDoubleVec & currents)
{
    for (size_t i=0; i<NumberOfActuators_; i++) {
        currents[i] = std::min(currents[i], ActuatorCurrentCommandLimits_[i]);
        currents[i] = std::max(currents[i], -ActuatorCurrentCommandLimits_[i]);
    }
}

void osaRobot1394::SetJointEffort(const vctDoubleVec & efforts)
{
    vctDoubleVec actuator_efforts(NumberOfActuators_);
    actuator_efforts = Configuration_.JointToActuatorEffort * efforts;
    this->SetActuatorEffort(actuator_efforts);
}

void osaRobot1394::SetActuatorEffort(const vctDoubleVec & efforts)
{
    // Convert efforts to bits and set the command
    vctDoubleVec clipped_efforts = efforts;
    vctDoubleVec currents(NumberOfActuators_);

    // this->clip_actuator_efforts(clipped_efforts);

    this->ActuatorEffortToCurrent(clipped_efforts, currents);
    this->SetActuatorCurrent(currents);
}

void osaRobot1394::SetActuatorCurrent(const vctDoubleVec & currents)
{
    // Convert amps to bits and set the command
    vctDoubleVec clipped_amps = currents;
    vctIntVec bits(NumberOfActuators_);

    this->ClipActuatorCurrent(clipped_amps);
    this->ActuatorCurrentToBits(clipped_amps, bits);
    this->SetActuatorCurrentBits(bits);

    // Store commanded amps
    ActuatorCurrentCommand_ = clipped_amps;
}

void osaRobot1394::SetActuatorCurrentBits(const vctIntVec & bits)
{
    // If we are currently calibrating current, don't apply anything
    if (CalibrateCurrentCommandOffsetRequested_) {
        return;
    }

    for (size_t i=0; i<NumberOfActuators_; i++) {
        ActuatorInfo_[i].board->SetMotorCurrent(ActuatorInfo_[i].axis, bits[i]);
    }

    // Store commanded bits
    ActuatorCurrentBitsCommand_ = bits;
}

void osaRobot1394::CalibrateCurrentCommandOffsetsRequest(const int & numberOfSamples)
{
    SetActuatorCurrent(vctDoubleVec(NumberOfActuators(), 0.0));
    CalibrateCurrentBufferIndex_ = 0;
    CalibrateCurrentCommandBuffers_.resize(numberOfSamples);
    CalibrateCurrentFeedbackBuffers_.resize(numberOfSamples);
    CalibrateCurrentBufferSize_ = numberOfSamples;
    CalibrateCurrentCommandOffsetRequested_ = true;
}

void osaRobot1394::CalibrateCurrentCommandOffsets(void)
{
    vctDoubleVec current_command_sums(NumberOfActuators_);
    vctDoubleVec current_feedback_sums(NumberOfActuators_);
    vctDoubleVec current_biases(NumberOfActuators_);

    // Compute current bias for each actuator
    for (size_t sample=0; sample < CalibrateCurrentBufferSize_; sample++) {
        current_command_sums = current_command_sums + CalibrateCurrentCommandBuffers_[sample];
        current_feedback_sums = current_feedback_sums + CalibrateCurrentFeedbackBuffers_[sample];
    }

    current_biases = (current_command_sums - current_feedback_sums) /
        static_cast<double>(CalibrateCurrentBufferSize_);

    // Store current bias
    current_biases.ElementwiseMultiply(CurrentToBitsScales_);
    CurrentToBitsOffsets_ = CurrentToBitsOffsets_ + current_biases;

    // Send the latest current command to the actuators to apply the bias
    this->SetActuatorCurrent(ActuatorCurrentCommand_);
}

void osaRobot1394::CalibrateEncoderOffsetsFromPots(void)
{
    vctDoubleVec joint_positions(NumberOfJoints_);
    vctDoubleVec joint_error(NumberOfJoints_);
    vctDoubleVec actuator_error(NumberOfActuators_);

    switch(PotType_) {

    case POTENTIOMETER_UNDEFINED:
        // TODO: Return error of some kind?
        // CMN_LOG_CLASS_INIT_ERROR << "ResetEncoderOffsetUsingPotPosSI: can't set encoder offset, potentiometer's position undefined";
        break;

    case POTENTIOMETER_ON_JOINTS:
        joint_positions = Configuration_.ActuatorToJointPosition * EncoderPosition_;
        joint_error = joint_positions - PotPosition_;
        actuator_error = Configuration_.JointToActuatorPosition * joint_error;
        BitsToPositionOffsets_ = BitsToPositionOffsets_ - actuator_error;
        break;

    case POTENTIOMETER_ON_ACTUATORS:
        actuator_error = EncoderPosition_ - PotPosition_;
        BitsToPositionOffsets_ = BitsToPositionOffsets_ - actuator_error;
        break;
    };
}

bool osaRobot1394::Valid(void) const {
    return Valid_;
}

bool osaRobot1394::PowerStatus(void) const {
    return PowerStatus_;
}

bool osaRobot1394::SafetyRelay(void) const {
    return SafetyRelay_;
}

bool osaRobot1394::WatchdogStatus(void) const {
    return WatchdogStatus_;
}

osaRobot1394Configuration osaRobot1394::GetConfiguration(void) const {
    return Configuration_;
}

std::string osaRobot1394::Name(void) const {
    return Name_;
}

double osaRobot1394::NumberOfJoints(void) const {
    return NumberOfJoints_;
}

double osaRobot1394::NumberOfActuators(void) const {
    return NumberOfActuators_;
}

void osaRobot1394::GetJointTypes(prmJointTypeVec & joint_types) const
{
    joint_types.resize(NumberOfJoints_);
    for (size_t i = 0; i < NumberOfJoints_; i++) {
        joint_types[i] = JointType_[i];
    }
}
void osaRobot1394::GetJointEffortCommandLimits(vctDoubleVec & limits) const
{
    limits = JointEffortCommandLimits_;
}

void osaRobot1394::GetActuatorEffortCommandLimits(vctDoubleVec & limits) const
{
    limits = ActuatorEffortCommandLimits_;
}

void osaRobot1394::GetActuatorCurrentCommandLimits(vctDoubleVec & limits) const
{
    limits = ActuatorCurrentCommandLimits_;
}




void osaRobot1394::EncoderPositionToBits(const vctDoubleVec & pos, vctIntVec & bits) const {
    for (size_t i = 0; i < bits.size() && i < pos.size(); i++) {
        bits[i] = static_cast<long>((pos[i] - BitsToPositionOffsets_[i]) / BitsToPositionScales_[i]);
    }
}
void osaRobot1394::EncoderBitsToPosition(const vctIntVec & bits, vctDoubleVec & pos) const {
    for (size_t i = 0; i < bits.size() && i < pos.size(); i++) {
        pos[i] = static_cast<double>(bits[i]) * BitsToPositionScales_[i] + BitsToPositionOffsets_[i];
    }
}
void osaRobot1394::EncoderBitsToDPosition(const vctIntVec & bits, vctDoubleVec & dpos) const {
    for (size_t i = 0; i < bits.size() && i < dpos.size(); i++) {
        dpos[i] = static_cast<double>(bits[i]) * BitsToDPositionScales_[i] + BitsToDPositionOffsets_[i];
    }
}
void osaRobot1394::EncoderBitsToDTime(const vctIntVec & bits, vctDoubleVec & dt) const {
    for (size_t i = 0; i < bits.size() && i < dt.size(); i++) {
        dt[i] = static_cast<double>(bits[i]) * BitsToDTimeScales_[i] + BitsToDTimeOffsets_[i];
    }
}
void osaRobot1394::EncoderBitsToVelocity(const vctIntVec & bits, vctDoubleVec & vel) const
{
    const int method = 1;
    switch(method){
    case 1:
        // estimation in FPGA
        // NOTE: BitsToVecocityScales, BitsToVelocityOffsets = 0
        for (size_t i = 0; i < bits.size() && i < vel.size(); i++) {
            vel[i] = BitsToDPositionScales_[i] / static_cast<double>(bits[i]);
            if (vel[i] < 0.01){
                vel[i] = 0.0;
            }
        }
        break;
    case 2:
        // diff averaged pot
        break;
    case 3:
        // savitzky-golay filtering
        break;
    default:
        std::cerr << "Unknown conversion methods chosen" << std::endl;
    }
}
void osaRobot1394::ActuatorEffortToCurrent(const vctDoubleVec & efforts, vctDoubleVec & currents) const {
    currents.ElementwiseProductOf(efforts, EffortToCurrentScales_);
}
void osaRobot1394::ActuatorCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const {
    for (size_t i = 0; i < bits.size() && i < currents.size(); i++) {
        bits[i] = static_cast<long>(currents[i] * CurrentToBitsScales_[i] + CurrentToBitsOffsets_[i]);
    }
}
void osaRobot1394::ActuatorBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const {
    for (size_t i = 0; i < bits.size() && i < currents.size(); i++) {
        currents[i] = static_cast<double>(bits[i]) * BitsToCurrentScales_[i] + BitsToCurrentOffsets_[i];
    }
}
void osaRobot1394::ActuatorCurrentToEffort(const vctDoubleVec & currents, vctDoubleVec & efforts) const {
    efforts.ElementwiseProductOf(currents, EffortToCurrentScales_);
}

void osaRobot1394::PotBitsToVoltage(const vctIntVec & bits, vctDoubleVec & voltages) const {
    for (size_t i = 0; i < bits.size() && i < voltages.size(); i++) {
        voltages[i] = static_cast<double>(bits[i]) * BitsToVoltageScales_[i] + BitsToVoltageOffsets_[i];
    }
}
void osaRobot1394::PotVoltageToPosition(const vctDoubleVec & voltages, vctDoubleVec & pos) const {
    pos.ElementwiseProductOf(voltages, VoltageToPositionScales_);
    pos.SumOf(pos, VoltageToPositionOffsets_);
}
