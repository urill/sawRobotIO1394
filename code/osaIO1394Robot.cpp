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

#include <sawRobotIO1394/osaIO1394Robot.h>

#include <AmpIO.h>

using namespace sawRobotIO1394;
using namespace osaIO1394;

osaIO1394Robot::osaIO1394Robot(const osaIO1394::RobotConfiguration & config,
                               const size_t max_consecutive_current_safety_violations,
                               const size_t actuator_current_buffer_size):
    // IO Structures
    Boards_(),
    UniqueBoards_(),
    // State Initialization
    Valid_(false),
    PowerStatus_(false),
    PreviousPowerStatus_(false),
    WatchdogStatus_(false),
    SafetyRelay_(false),
    CalibrateCurrentCommandOffsetRequested_(false),
    CurrentSafetyViolationsCounter_(0),
    CurrentSafetyViolationsMaximum_(max_consecutive_current_safety_violations),
    CalibrateCurrentBufferSize_(actuator_current_buffer_size),
    CalibrateCurrentBufferIndex_(0),
    CalibrateCurrentCommandBuffers_(NumberOfActuators_)
{
    this->Configure(config);
}

void osaIO1394Robot::Configure(const osaIO1394::RobotConfiguration & config)
{
    // Store the configuration
    Configuration_ = config;

    //  info
    Name_ = config.Name;
    NumberOfActuators_ = config.NumberOfActuators;
    NumberOfJoints_ = config.NumberOfJoints;
    PotType_ = config.PotLocation;

    // Low-level API
    Boards_.resize(NumberOfActuators_);

    // Initialize state vectors to the appropriate sizes
    ActuatorPowerStatus_.resize(NumberOfActuators_);
    ActuatorPowerEnabled_.resize(NumberOfActuators_);
    DigitalInputs_.resize(NumberOfActuators_);
    PotBits_.resize(NumberOfActuators_);
    EncoderPositionBits_.resize(NumberOfActuators_);
    EncoderVelocityBits_.resize(NumberOfActuators_);
    ActuatorCurrentBitsCommand_.resize(NumberOfActuators_);
    ActuatorCurrentBitsFeedback_.resize(NumberOfActuators_);
    PotVoltage_.resize(NumberOfActuators_);
    PotPosition_.resize(NumberOfActuators_);
    EncoderPosition_.resize(NumberOfActuators_);
    EncoderVelocity_.resize(NumberOfActuators_);
    JointPosition_.resize(NumberOfJoints_);
    JointVelocity_.resize(NumberOfJoints_);
    ActuatorCurrentCommand_.resize(NumberOfActuators_);
    ActuatorEffortCommand_.resize(NumberOfActuators_);
    ActuatorCurrentFeedback_.resize(NumberOfActuators_);
    ActuatorEffortFeedback_.resize(NumberOfActuators_);

    // Initialize property vectors to the appropriate sizes
    BoardAxes_.resize(NumberOfActuators_);
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
        const osaIO1394::ActuatorConfiguration & actuator = config.Actuators[i];
        const osaIO1394::DriveConfiguration & drive = actuator.Drive;
        const osaIO1394::EncoderConfiguration & encoder = actuator.Encoder;
        const osaIO1394::PotConfiguration & pot = actuator.Pot;

        BoardAxes_[i] = actuator.AxisID;
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
        ActuatorCurrentCommand_[i] = 0.0;
        ActuatorCurrentFeedback_[i] = 0.0;
    }

    // Compute effort command limits
    JointEffortCommandLimits_ = Configuration_.ActuatorToJointEffort * ActuatorEffortCommandLimits_;
}

void osaIO1394Robot::SetBoards(std::vector<AmpIO*> boards)
{
    if (boards.size() != NumberOfActuators_) {
        throw osaIO1394::configuration_error("Number of boards different than the number of actuators.");
    }

    for (size_t i = 0; i < NumberOfActuators_; i++) {
        // Store this board
        Boards_[i] = boards[i];
        // Construct a list of unique boards
        UniqueBoards_[Boards_[i]->GetBoardId()] = Boards_[i];
    }
}

void osaIO1394Robot::PollValidity(void)
{
    // Make sure the boards have been configured
    if (NumberOfActuators_ != Boards_.size()) {
        throw osaIO1394::configuration_error("Number of boards different than the number of actuators.");
    }

    // Store previous state
    PreviousPowerStatus_ = PowerStatus_;

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
        throw osaIO1394::hardware_error("Boards invalid.");
    }
}

void osaIO1394Robot::PollState(void)
{
    // Poll data
    for (size_t i = 0; i < NumberOfActuators_; i++) {
        AmpIO * board = Boards_[i];
        int axis = BoardAxes_[i];

        if (!board || (axis < 0)) continue; // We probably don't need this check any more

        DigitalInputs_[i] = board->GetDigitalInput();

        EncoderPositionBits_[i] = ((int)(board->GetEncoderPosition(axis) << 8)) >> 8; // convert from 24 bits signed stored in 32 unsigned to 32 signed
        EncoderVelocityBits_[i] = ((int)(board->GetEncoderVelocity(axis) << 16)) >> 16; // convert from 16 bits signed stored in 32 unsigned to 32 signed

        PotBits_[i] = board->GetAnalogInput(axis);

        ActuatorCurrentBitsFeedback_[i] = board->GetMotorCurrent(axis);
        ActuatorPowerEnabled_[i] = board->GetAmpEnable(axis);
        ActuatorPowerStatus_[i] = board->GetAmpStatus(axis);

        // first temperature corresponds to first 2 actuators, second to last 2
        // board reports temperature in celsius * 2
        Temperature_[i] = (board->GetAmpTemperature(axis / 2)) / 2.0;
    }
}

void osaIO1394Robot::ConvertState(void)
{
    // Perform read conversions
    EncoderBitsToPosition(EncoderPositionBits_, EncoderPosition_);
    JointPosition_ = Configuration_.ActuatorToJointPosition * EncoderPosition_;

    EncoderBitsToVelocity(EncoderVelocityBits_, EncoderVelocity_);
    JointVelocity_ = Configuration_.ActuatorToJointPosition * EncoderVelocity_;

    ActuatorBitsToCurrent(ActuatorCurrentBitsFeedback_, ActuatorCurrentFeedback_);
    ActuatorCurrentToEffort(ActuatorCurrentFeedback_, ActuatorEffortFeedback_);

    PotBitsToVoltage(PotBits_, PotVoltage_);
    PotVoltageToPosition(PotVoltage_, PotPosition_);
}

void osaIO1394Robot::CheckState(void)
{
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
        throw osaIO1394::safety_error("Too many consecutive current safety violations.  power has been disabled.");
    }
}

void osaIO1394Robot::EnablePower(void)
{
    this->EnableBoardsPower();
    this->SetActuatorPower(true);
}

void osaIO1394Robot::EnableBoardsPower(void)
{
    for (unique_board_iterator board = UniqueBoards_.begin();
         board != UniqueBoards_.end();
         ++board) {
        board->second->WriteSafetyRelay(true);
        board->second->WritePowerEnable(true);
    }
    this->SetActuatorPower(true);
}

void osaIO1394Robot::DisablePower(void)
{
    this->DisableBoardPower();
    this->SetActuatorPower(false);
}

void osaIO1394Robot::DisableBoardPower(void)
{
    for (unique_board_iterator board = UniqueBoards_.begin();
         board != UniqueBoards_.end();
         ++board) {
        board->second->WritePowerEnable(false);
        board->second->WriteSafetyRelay(false);
    }
}

void osaIO1394Robot::SetSafetyRelay(const bool & enabled)
{
    for (unique_board_iterator board = UniqueBoards_.begin();
         board != UniqueBoards_.end();
         ++board) {
        board->second->SetSafetyRelay(enabled);
    }
}

void osaIO1394Robot::SetWatchdogPeriod(const double & periodInSeconds)
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

void osaIO1394Robot::SetActuatorPower(const bool & enabled)
{
    for (size_t i = 0; i < NumberOfActuators_; i++) {
        Boards_[i]->SetAmpEnable(BoardAxes_[i], enabled);
    }
}

void osaIO1394Robot::SetActuatorPower(const vctBoolVec & enabled)
{
    for (size_t i = 0; i < NumberOfActuators_; i++) {
        Boards_[i]->SetAmpEnable(BoardAxes_[i], enabled[i]);
    }
}

void osaIO1394Robot::SetEncoderPosition(const vctDoubleVec & pos)
{
    vctIntVec bits(NumberOfActuators_);
    this->EncoderPositionToBits(pos, bits);
    this->SetEncoderPositionBits(bits);
}

void osaIO1394Robot::SetEncoderPositionBits(const vctIntVec & bits)
{
    for (size_t i = 0; i < NumberOfActuators_; i++) {
        Boards_[i]->WriteEncoderPreload(BoardAxes_[i], bits[i]);
    }
}

void osaIO1394Robot::SetSingleEncoderPosition(const int index, const double pos)
{
    SetSingleEncoderPositionBits(index, (pos - BitsToPositionOffsets_[index]) / BitsToPositionScales_[index]);
}

void osaIO1394Robot::SetSingleEncoderPositionBits(const int index, const int bits)
{
    Boards_[index]->WriteEncoderPreload(BoardAxes_[index], bits);
}

void osaIO1394Robot::ClipActuatorEffort(vctDoubleVec & efforts)
{
    for (size_t i=0; i<NumberOfActuators_; i++) {
        efforts[i] = std::min(efforts[i], ActuatorEffortCommandLimits_[i]);
        efforts[i] = std::max(efforts[i], -ActuatorEffortCommandLimits_[i]);
    }
}

void osaIO1394Robot::ClipActuatorCurrent(vctDoubleVec & currents)
{
    for (size_t i=0; i<NumberOfActuators_; i++) {
        currents[i] = std::min(currents[i], ActuatorCurrentCommandLimits_[i]);
        currents[i] = std::max(currents[i], -ActuatorCurrentCommandLimits_[i]);
    }
}

void osaIO1394Robot::SetJointEffort(const vctDoubleVec & efforts)
{
    vctDoubleVec actuator_efforts(NumberOfActuators_);
    actuator_efforts = Configuration_.JointToActuatorEffort * efforts;
    this->SetActuatorEffort(actuator_efforts);
}

void osaIO1394Robot::SetActuatorEffort(const vctDoubleVec & efforts)
{
    // Convert efforts to bits and set the command
    vctDoubleVec clipped_efforts = efforts;
    vctDoubleVec currents(NumberOfActuators_);

    // this->clip_actuator_efforts(clipped_efforts);

    this->ActuatorEffortToCurrent(clipped_efforts, currents);
    this->SetActuatorCurrent(currents);
}

void osaIO1394Robot::SetActuatorCurrent(const vctDoubleVec & currents)
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

void osaIO1394Robot::SetActuatorCurrentBits(const vctIntVec & bits)
{
    for (size_t i=0; i<NumberOfActuators_; i++) {
        Boards_[i]->SetMotorCurrent(BoardAxes_[i], bits[i]);
    }

    // Store commanded bits
    ActuatorCurrentBitsCommand_ = bits;
}

void osaIO1394Robot::CalibrateCurrentCommandOffsetsRequest(const int & numberOfSamples)
{
    CalibrateCurrentBufferIndex_ = 0;
    CalibrateCurrentCommandBuffers_.resize(numberOfSamples);
    CalibrateCurrentFeedbackBuffers_.resize(numberOfSamples);
    CalibrateCurrentBufferSize_ = numberOfSamples;
    CalibrateCurrentCommandOffsetRequested_ = true;
}

void osaIO1394Robot::CalibrateCurrentCommandOffsets(void)
{
    vctDoubleVec
        current_command_sums(NumberOfActuators_),
        current_feedback_sums(NumberOfActuators_),
        current_biases(NumberOfActuators_);

    // Copute current bias for each actuator
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

void osaIO1394Robot::CalibrateEncoderOffsetsFromPots(void)
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

bool osaIO1394Robot::Valid(void) const {
    return Valid_;
}

bool osaIO1394Robot::PowerStatus(void) const {
    return PowerStatus_;
}

bool osaIO1394Robot::SafetyRelay(void) const {
    return SafetyRelay_;
}

bool osaIO1394Robot::WatchdogStatus(void) const {
    return WatchdogStatus_;
}

osaIO1394::RobotConfiguration osaIO1394Robot::GetConfiguration(void) const {
    return Configuration_;
}

std::string osaIO1394Robot::Name(void) const {
    return Name_;
}

double osaIO1394Robot::NumberOfJoints(void) const {
    return NumberOfJoints_;
}

double osaIO1394Robot::NumberOfActuators(void) const {
    return NumberOfActuators_;
}

void osaIO1394Robot::GetJointTypes(prmJointTypeVec & joint_types) const
{
    joint_types.resize(NumberOfJoints_);
    for (size_t i = 0; i < NumberOfJoints_; i++) {
        joint_types[i] = JointType_[i];
    }
}
void osaIO1394Robot::GetJointEffortCommandLimits(vctDoubleVec & limits) const
{
    limits = JointEffortCommandLimits_;
}

void osaIO1394Robot::GetActuatorEffortCommandLimits(vctDoubleVec & limits) const
{
    limits = ActuatorEffortCommandLimits_;
}

void osaIO1394Robot::GetActuatorCurrentCommandLimits(vctDoubleVec & limits) const
{
    limits = ActuatorCurrentCommandLimits_;
}

#ifndef SAW_ROBOT_IO_1394_WO_CISST

void osaIO1394Robot::EncoderPositionToBits(const vctDoubleVec & pos, vctIntVec & bits) const {
    for (size_t i = 0; i < bits.size() && i < pos.size(); i++) {
        bits[i] = static_cast<long>((pos[i] - BitsToPositionOffsets_[i]) / BitsToPositionScales_[i]);
    }
}
void osaIO1394Robot::EncoderBitsToPosition(const vctIntVec & bits, vctDoubleVec & pos) const {
    for (size_t i = 0; i < bits.size() && i < pos.size(); i++) {
        pos[i] = static_cast<double>(bits[i]) * BitsToPositionScales_[i] + BitsToPositionOffsets_[i];
    }
}
void osaIO1394Robot::EncoderBitsToDPosition(const vctIntVec & bits, vctDoubleVec & dpos) const {
    for (size_t i = 0; i < bits.size() && i < dpos.size(); i++) {
        dpos[i] = static_cast<double>(bits[i]) * BitsToDPositionScales_[i] + BitsToDPositionOffsets_[i];
    }
}
void osaIO1394Robot::EncoderBitsToDTime(const vctIntVec & bits, vctDoubleVec & dt) const {
    for (size_t i = 0; i < bits.size() && i < dt.size(); i++) {
        dt[i] = static_cast<double>(bits[i]) * BitsToDTimeScales_[i] + BitsToDTimeOffsets_[i];
    }
}
void osaIO1394Robot::EncoderBitsToVelocity(const vctIntVec & bits, vctDoubleVec & vel) const {
    for (size_t i = 0; i < bits.size() && i < vel.size(); i++) {
        vel[i] = static_cast<double>(bits[i]) * BitsToVecocityScales_[i] + BitsToVelocityOffsets_[i];
    }
}
void osaIO1394Robot::ActuatorEffortToCurrent(const vctDoubleVec & efforts, vctDoubleVec & currents) const {
    currents = currents.ElementwiseProductOf(efforts, EffortToCurrentScales_);
}
void osaIO1394Robot::ActuatorCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const {
    for (size_t i = 0; i < bits.size() && i < currents.size(); i++) {
        bits[i] = static_cast<long>(currents[i] * CurrentToBitsScales_[i] + CurrentToBitsOffsets_[i]);
    }
}
void osaIO1394Robot::ActuatorBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const {
    for (size_t i = 0; i < bits.size() && i < currents.size(); i++) {
        currents[i] = static_cast<double>(bits[i]) * BitsToCurrentScales_[i] + BitsToCurrentOffsets_[i];
    }
}
void osaIO1394Robot::ActuatorCurrentToEffort(const vctDoubleVec & currents, vctDoubleVec & efforts) const {
    efforts = efforts.ElementwiseProductOf(currents, EffortToCurrentScales_);
}

void osaIO1394Robot::PotBitsToVoltage(const vctIntVec & bits, vctDoubleVec & voltages) const {
    for (size_t i = 0; i < bits.size() && i < voltages.size(); i++) {
        voltages[i] = static_cast<double>(bits[i]) * BitsToVoltageScales_[i] + BitsToVoltageOffsets_[i];
    }
}
void osaIO1394Robot::PotVoltageToPosition(const vctDoubleVec & voltages, vctDoubleVec & pos) const {
    pos.ElementwiseProductOf(voltages, VoltageToPositionScales_);
    pos.SumOf(pos, VoltageToPositionOffsets_);
}

#else // ifndef SAW_ROBOT_IO_1394_WO_CISST

void osaIO1394::encoder_pos_to_bits(const vctDoubleVec & pos, vctIntVec & bits) const {
    bits = (pos - bits_to_pos_offsets_).cwiseQuotient(bits_to_pos_ratios_).cast<int>();
}
void osaIO1394::encoder_bits_to_pos(const vctIntVec & bits, vctDoubleVec & pos) const {
    pos = bits.cast<double>().cwiseProduct(bits_to_pos_ratios_) + bits_to_pos_offsets_;
}
void osaIO1394::encoder_bits_to_dpos(const vctIntVec & bits, vctDoubleVec & dpos) const {
    dpos = bits.cast<double>().cwiseProduct(bits_to_dpos_ratios_) + bits_to_dpos_offsets_;
}
void osaIO1394::encoder_bits_to_dt(const vctIntVec & bits, vctDoubleVec & dt) const {
    dt = bits.cast<double>().cwiseProduct(bits_to_dt_ratios_) + bits_to_dt_offsets_;
}
void osaIO1394::encoder_bits_to_vel(const vctIntVec & bits, vctDoubleVec & vel) const {
    vel = bits.cast<double>().cwiseProduct(bits_to_volts_ratios_) + bits_to_vel_offsets_;
}

void osaIO1394::actuator_bits_to_amps(const vctIntVec & bits, vctDoubleVec & currents) const {
    currents = bits.cast<double>().cwiseProduct(bits_to_amps_ratios_) + bits_to_amps_offsets_;
}
void osaIO1394::actuator_amps_to_bits(const vctDoubleVec & currents, vctIntVec & bits) const {
    bits = (currents.cwiseProduct(amps_to_bits_ratios_) + amps_to_bits_offsets_).cast<uint32_t>();
}
void osaIO1394::actuator_efforts_to_amps(const vctDoubleVec & efforts, vctDoubleVec & currents) const {
    currents = efforts.cwiseProduct(efforts_to_amps_ratios_);
}
void osaIO1394::actuator_amps_to_efforts(const vctDoubleVec & currents, vctDoubleVec & efforts) const {
    efforts = currents.cwiseQuotient(efforts_to_amps_ratios_);
}

void osaIO1394::pot_bits_to_volts(const vctIntVec & bits, vctDoubleVec & voltages) const {
    voltages = bits.cast<double>().cwiseProduct(bits_to_volts_ratios_) + bits_to_volts_offsets_;
}
void osaIO1394::pot_volts_to_pos(const vctDoubleVec & voltages, vctDoubleVec & pos) const {
    pos = voltages.cwiseProduct(volts_to_pos_ratios_) + volts_to_pos_offsets_;
}

#endif // ifndef SAW_ROBOT_IO_1394_WO_CISST
