/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaRobot1394_h
#define _osaRobot1394_h

#include <vector>
#include <map>
#include <stdint.h>

#ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstParameterTypes/prmJointType.h>
#else
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "EigenWrapper.h"
#include "MinimalPrm.h"
#endif

#include <sawRobotIO1394/osaConfiguration1394.h>

class AmpIO;

namespace sawRobotIO1394 {

    /**
     * IO1394 Robot Abstraction Layer
     * This class is responsible for robot-level actuation and unit
     * conversion as well as safety monitoring for the QLA robot control
     * architecture.
     **/
class osaRobot1394 {

    public:
        //! Watchdog counts per ms (note counter width, e.g. 16 bits)
        static const size_t WATCHDOG_MS_TO_COUNT = 192;

        /** \name Lifecycle
         *\{**/
        osaRobot1394(const osaRobot1394Configuration & config,
                     const size_t max_consecutive_current_safety_violations = 100);

        void Configure(const osaRobot1394Configuration & config);

        void SetBoards(const std::vector<osaActuatorMapping> & actuatorBoards,
                       const std::vector<osaBrakeMapping> & brakeBoards);

        /**}**/

        /** \name State Update Functions
         * These functions interact with the lower-level hardware to query
         * information only and update this class' members.
         *\{**/
        void PollValidity(void);
        void PollState(void);
        void ConvertState(void);
        void CheckState(void);
        /**}**/

        /** \name Command Functions
         * These functions interact with the lower-level hardware when called to
         * change its state in some way. Note that these functions do not have
         * any side-effects in the class.
         *\{**/
        //
        //! Power / Safety Control
        void EnablePower(void);
        void EnableBoardsPower(void);
        void DisablePower(void);
        void DisableBoardPower(void);
        void SetSafetyRelay(const bool & enabled);
        void SetWatchdogPeriod(const double & periodInSeconds);

        void SetActuatorPower(const bool & enabled);
        void SetActuatorPower(const vctBoolVec & enabled);
        void SetBrakePower(const bool & enabled);
        void SetBrakePower(const vctBoolVec & enabled);

        //! Encoder Control
        void SetEncoderPosition(const vctDoubleVec & pos);
        void SetEncoderPositionBits(const vctIntVec & bits);
        void SetSingleEncoderPosition(const int index, const double pos = 0);
        void SetSingleEncoderPositionBits(const int index, const int bits = 0);

        //! Actuator Control
        void SetJointEffort(const vctDoubleVec & efforts);
        void SetActuatorEffort(const vctDoubleVec & efforts);
        void SetActuatorCurrent(const vctDoubleVec & currents);
        void SetActuatorCurrentBits(const vctIntVec & bits);

        //! Brake Control
        void SetBrakeCurrent(const vctDoubleVec & currents);
        void SetBrakeCurrentBits(const vctIntVec & bits);
        void SetBrakeReleaseCurrent(void);
        void SetBrakeReleasedCurrent(void);
        void SetBrakeEngagedCurrent(void);
        /**}**/


        /** \name State Accessors
         * These accessors only access data which is contained in this class, i.e.
         * they do not interact with the lower-level hardware. To update these data
         * from the lower-level system, you must call \ref poll_state.
         *\{**/
        bool Valid(void) const;
        bool PowerStatus(void) const;
        bool SafetyRelay(void) const;
        bool WatchdogStatus(void) const;
        const vctBoolVec & ActuatorPowerStatus(void) const;
        const vctBoolVec & BrakePowerStatus(void) const;
        const vctDoubleVec & ActuatorCurrentFeedback(void) const;
        const vctDoubleVec & BrakeCurrentFeedback(void) const;
        const vctDoubleVec & PotPosition(void) const;
        const vctDoubleVec & ActuatorTimeStamp(void) const;
        const vctDoubleVec & BrakeTimeStamp(void) const;
        const vctDoubleVec & EncoderPosition(void) const;
        const vctDoubleVec & EncoderVelocity(void) const;
        /**}**/

        /** \name Parameter Accessors
         *\{**/
        osaRobot1394Configuration GetConfiguration(void) const;
        std::string Name(void) const;
        size_t NumberOfJoints(void) const;
        size_t NumberOfActuators(void) const;
        size_t NumberOfBrakes(void) const;
        void GetJointTypes(prmJointTypeVec & jointTypes) const;
        void GetJointEffortCommandLimits(vctDoubleVec & limits) const;
        void GetActuatorEffortCommandLimits(vctDoubleVec & limits) const;
        void GetActuatorCurrentCommandLimits(vctDoubleVec & limits) const;
        /**}**/

        /** \name Bias Calibration Functions
         *\{**/
        void CalibrateEncoderOffsetsFromPots(void);
        /**}**/

        /** \name Conversion Functions
         * These functions convert data between units for various purposes. They
         * have no side-effects.
         *\{**/
        //! Conversions for encoders
        void EncoderPositionToBits(const vctDoubleVec & pos, vctIntVec & bits) const;
        void EncoderBitsToPosition(const vctIntVec & bits, vctDoubleVec & pos) const;
        void EncoderBitsToDPosition(const vctIntVec & bits, vctDoubleVec & dpos) const;
        void EncoderBitsToDTime(const vctIntVec & bits, vctDoubleVec & dt) const;
        void EncoderBitsToVelocity(const vctIntVec & bits, vctDoubleVec & vel) const;

        //! Conversions for actuator current commands and measurements
        void ActuatorEffortToCurrent(const vctDoubleVec & efforts, vctDoubleVec & currents) const;
        void ActuatorCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const;
        void ActuatorBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const;
        void ActuatorCurrentToEffort(const vctDoubleVec & currents, vctDoubleVec & efforts) const;

        //! Conversions for brake commands
        void BrakeCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const;
        void BrakeBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const;

        //! Conversions for potentiometers
        void PotBitsToVoltage(const vctIntVec & bits, vctDoubleVec & voltages) const;
        void PotVoltageToPosition(const vctDoubleVec & voltages, vctDoubleVec & pos) const;
        /**}**/

    protected:

        void ClipActuatorEffort(vctDoubleVec & efforts);
        void ClipActuatorCurrent(vctDoubleVec & currents);
        void ClipBrakeCurrent(vctDoubleVec & currents);

        //! Board Objects
        std::vector<osaActuatorMapping> mActuatorInfo;
        std::vector<osaBrakeMapping> mBrakeInfo;
        std::map<int, AmpIO*> mUniqueBoards;
        typedef std::map<int, AmpIO*>::iterator unique_board_iterator;
        typedef std::map<int, AmpIO*>::const_iterator unique_board_const_iterator;

        //! Robot Configuration
        osaRobot1394Configuration mConfiguration;
        std::string mName;
        size_t mNumberOfActuators;
        size_t mNumberOfJoints;
        size_t mNumberOfBrakes;

        //! Vectors of actuator properties
        vctIntVec
            mCountsPerTurn;

        vctDoubleVec
            mEffortToCurrentScales,
            mActuatorCurrentToBitsScales,
            mBrakeCurrentToBitsScales,
            mActuatorCurrentToBitsOffsets,
            mBrakeCurrentToBitsOffsets,
            mActuatorBitsToCurrentScales,
            mBrakeBitsToCurrentScales,
            mActuatorBitsToCurrentOffsets,
            mBrakeBitsToCurrentOffsets,
            mBitsToPositionScales,
            mBitsToPositionOffsets,
            mBitsToDPositionScales,
            mBitsToDPositionOffsets,
            mBitsToDTimeScales,
            mBitsToDTimeOffsets,
            mBitsToVecocityScales,
            mBitsToVelocityOffsets,
            mBitsToVoltageScales,
            mBitsToVoltageOffsets,
            mVoltageToPositionScales,
            mVoltageToPositionOffsets;

        vctDoubleVec
            mJointEffortCommandLimits,
            mActuatorEffortCommandLimits,
            mActuatorCurrentCommandLimits,
            mBrakeCurrentCommandLimits,
            mActuatorCurrentFeedbackLimits, // limit used to trigger error
            mBrakeCurrentFeedbackLimits;    // limit used to trigger error

        //! Robot type
        prmJointTypeVec mJointType;
        osaPot1394Location mPotType;

        //! State Members
        bool
            mValid,
            mPowerStatus,
            mPreviousPowerStatus,
            mWatchdogStatus,
            mPreviousWatchdogStatus,
            mIsAllBoardsFirmWareFour;

        unsigned short mSafetyRelay;

        vctBoolVec
            mActuatorPowerStatus,
            mBrakePowerStatus,
            mActuatorPowerEnabled,
            mBrakePowerEnabled,
            mDigitalInputs;

        vctIntVec
            mPotBits,
            mEncoderPositionBits,
            mEncoderVelocityBits,     // latched velocity
            mEncoderVelocityBitsNow,  // current counting velocity bits
            mEncoderDPositionBits,
            mEncoderDTimeBits;

        vctIntVec
            mActuatorCurrentBitsCommand,
            mBrakeCurrentBitsCommand,
            mActuatorCurrentBitsFeedback,
            mBrakeCurrentBitsFeedback;

        vctDoubleVec
            mActuatorTimeStamp,
            mBrakeTimeStamp,
            mPotVoltage,
            mPotPosition,
            mEncoderPosition,
            mEncoderPositionPrev,
            mEncoderVelocity,
            mEncoderVelocityDxDt,
            mEncoderDPosition,
            mEncoderDTime,
            mJointPosition,
            mJointVelocity,
            mActuatorCurrentCommand,
            mBrakeCurrentCommand,
            mActuatorEffortCommand,
            mActuatorCurrentFeedback,
            mBrakeCurrentFeedback,
            mActuatorEffortFeedback,
            mActuatorTemperature,
            mBrakeTemperature,
            mBrakeReleaseCurrent,
            mBrakeReleaseTime,
            mBrakeReleasedCurrent,
            mBrakeEngagedCurrent;

        size_t
            mCurrentSafetyViolationsCounter,
            mCurrentSafetyViolationsMaximum;
    };

} // namespace sawRobotIO1394

#endif // _osaRobot1394_h
