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

#ifndef _RobotInternal_h_
#define _RobotInternal_h_

#include <string>
#include <vector>

#include <cisstCommon/cmnXMLPath.h>

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>

#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>

class mtsInterfaceProvided;
class mtsStateTable;
class AmpIO;

/*!
  \todo add configuration for potentiometer coupling to joints (default) or actuators
  \todo add software check to monitor current feedback vs. requested current
  \todo add check on configuration files to make sure all gains are non zero
  \todo add code to load units from config files and apply properly
  \todo create vectors for all data used in computations instead of digging into the vector of structures used to configure the robot.
  \todo create one state table per robot
  \todo create one state table for all digital inputs
  \todo use cisst naming conventions
  \todo create temporary vectors used for runtime computation in the object but add a dummy struct e.g. Robot.Temp.CurrentInA to clearly identify these as non-state
  \todo sort commands between joint vs actuator interfaces
  \todo test JSON config files after JSON branch merge
*/
class mtsRobotIO1394::RobotInternal {

public:
    /*! Defines where the potentiometers are positioned, if any. */
    typedef enum {POTENTIOMETER_UNDEFINED, POTENTIOMETER_ON_ACTUATORS, POTENTIOMETER_ON_JOINTS} PotentiometerType;

protected:

    /*! Pointer on existing services.  This allows to use the class
      name and level of detail of another class, e.g. the class that
      owns this map.  To set the "Owner", use the method SetOwner
      after the cmnNamedMap is constructed. */
    const cmnClassServicesBase * OwnerServices;

    /*! Method use to emulate the cmnGenericObject interface used by
      CMN_LOG_CLASS macros. */
    //@{
    inline const cmnClassServicesBase * Services(void) const {
        return this->OwnerServices;
    }

    inline cmnLogger::StreamBufType * GetLogMultiplexer(void) const {
        return cmnLogger::GetMultiplexer();
    }
    //@}

    //Nested class stores Actuator-Axis info
    class ActuatorInfo {
      public:
        struct Drive {
            double AmpsToBitsScale;
            double AmpsToBitsOffset;
            double BitsToFeedbackAmpsScale;
            double BitsToFeedbackAmpsOffset;
            double NmToAmpsScale;
            double MaxCurrentValue;
        };
        struct Encoder {
            double BitsToPosSIScale;
            double BitsToPosSIOffset;
            double BitsToDeltaPosSIScale;
            double BitsToDeltaPosSIOffset;
            double BitsToDeltaTScale;
            double BitsToDeltaTOffset;
            int CountsPerTurnValue;
        };
        struct AnalogIn {
            double BitsToVoltsScale;
            double BitsToVoltsOffset;
            double VoltsToPosSIScale;
            double VoltsToPosSIOffset;
        };
        AmpIO *board;
        int axisid;
        std::string primaryPosSensor;
        std::string secondaryPosSensor;
        Drive drive;
        Encoder encoder;
        AnalogIn analogIn;

      public:
        ActuatorInfo();
        ActuatorInfo(AmpIO *bptr, int aid);
        ~ActuatorInfo();
    };
    std::string robotName;              // Robot name (from config file)
    std::vector<ActuatorInfo> ActuatorList;   // Actuator information
    std::vector<AmpIO *> OwnBoards;           // Pointers to boards "owned" by this robot
    int NumberOfActuators;
    int NumberOfJoints;
    double TaskPeriod;
    bool HasActuatorToJointCoupling;
    PotentiometerType Potentiometers;

    /*! Struct used to scope data for bias amplifiers bit offset based on
      current feedback. */
    struct AmpsToBitsOffsetUsingFeedbackAmpsStruct {
        inline AmpsToBitsOffsetUsingFeedbackAmpsStruct(void):
            Count(0)
        {}
        size_t Count;
        vctDynamicVector<vctDoubleVec> ControlCurrents;
        vctDynamicVector<vctDoubleVec> FeedbackCurrents;
    } AmpsToBitsOffsetUsingFeedbackAmps;

    /*! Struct used to scope all configuration data used for computations,
      these vectors should be updated right after the configuration file is
      loaded. */
    struct {
        vctDoubleVec MotorCurrentMax;
    } Configuration;

    // State data
    bool           Valid;
    bool           PowerStatus;
    unsigned short SafetyRelay;
    bool           WatchdogTimeout;
    vctBoolVec     ampStatus;           // Amplifier actual status (ON or FAULT)
    vctBoolVec     ampEnable;           // Current amplifier enable state (read from boards)
    vctIntVec      encPosRaw;
    vctDoubleVec   PositionJoint;
    prmPositionJointGet PositionJointGet;
    prmPositionJointGet PositionActuatorGet;
    vctLongVec     encVelRaw;
    vctDoubleVec   encVel;
    vctLongVec     analogInRaw;
    vctDoubleVec   analogInVolts;
    vctDoubleVec   analogInPosSI;
    vctLongVec     motorFeedbackCurrentRaw;
    vctDoubleVec   motorFeedbackCurrent;
    prmForceTorqueJointSet TorqueJoint;
    vctDoubleVec   jointTorque;
    vctDoubleVec   jointTorqueMax;
    vctLongVec     motorControlCurrentRaw;
    vctDoubleVec   motorControlCurrent;
    vctDoubleVec   motorControlTorque;
    vctIntVec      encSetPosRaw;
    vctDoubleVec   encSetPos;

    vctDoubleMat ActuatorToJointPosition;
    vctDoubleMat JointToActuatorPosition;
    vctDoubleMat ActuatorToJointTorque;
    vctDoubleMat JointToActuatorTorque;

    // Convenient data for enabling/disabling amps
    vctBoolVec AllOn;
    vctBoolVec AllOff;

    // Methods for provided interface
    void GetNumberOfJoints(int & placeHolder) const;
    void EnablePower(void);
    void DisablePower(void);
    void EnableBoardsPower(void);
    void DisableBoardsPower(void);
    void EnableSafetyRelay(void);
    void DisableSafetyRelay(void);
    void SetWatchdogPeriod(const double &period_sec);
    void SetAmpEnable(const vctBoolVec &ampControl);
    void SetTorqueJoint(const prmForceTorqueJointSet & torques);
    void GetTorqueJointMax(vctDoubleVec & maxTorques) const;
    void SetMotorCurrentRaw(const vctLongVec &mcur);
    void SetMotorCurrent(const vctDoubleVec &mcur);
    void GetMotorCurrentMax(vctDoubleVec & placeHolder) const;
    void RequestAmpsToBitsOffsetUsingFeedbackAmps(const mtsInt & numberOfSamples);
    void ResetAmpsToBitsOffsetUsingFeedbackAmps(void);
    void ResetEncoderOffsetUsingPotPosSI(void);
    void ResetSingleEncoder(const int & actuatorIndex);
    void SetEncoderPositionRaw(const vctIntVec & epos);
    void SetEncoderPosition(const vctDoubleVec & epos);

    // Unit conversion Raw -- SI (partial implementation)
    void EncoderRawToSI(const vctIntVec &fromData, vctDoubleVec &toData) const;
    void EncoderSIToRaw(const vctDoubleVec &fromData, vctIntVec &toData) const;
    void EncoderRawToDeltaPosSI(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void EncoderRawToDeltaPosT(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void DriveAmpsToBits(const vctDoubleVec &fromData, vctLongVec &toData) const;
    void DriveBitsToFeedbackAmps(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void DriveNmToAmps(const vctDoubleVec &fromData, vctDoubleVec &toData) const;
    void DriveAmpsToNm(const vctDoubleVec &fromData, vctDoubleVec &toData) const;
    void AnalogInBitsToVolts(const vctLongVec &fromData, vctDoubleVec &toData) const;
    void AnalogInVoltsToPosSI(const vctDoubleVec &fromData, vctDoubleVec &toData) const;

    // Internal methods for configuring coupling
    void ConfigureCoupling (cmnXMLPath & xmlConfigFile, int robotNumber);
    void ConfigureCouplingMatrix(cmnXMLPath &xmlConfigFile, int robotNumber, const char *couplingString,
                                 int numRows, int numCols, vctDoubleMat &resultMatrix);

    // Functions for events
    struct {
        mtsFunctionWrite PowerStatus;
    } EventTriggers;

    /*! Update internal configuration.  Configuration uses vectors of structs
      to match the configuration files structure but this is inconvenient at
      runtime so we copy all the parameters in vectors indexed by actuator or
      joint index. */
    void UpdateInternalConfiguration(void);

    /*! Update the default values for the current and torques on actuators and
      joints. This method is called at the end of configuration to make sure
      the internal data members are set to 0 and the IO boards are reset
      properly. */
    void UpdateTorqueCurrentDefaults(void);

    /*! Compute the Joint maximum torques allowed based on actuator maximum
      torques and coupling matrices (if provided).  This method is called at
      the end of configuration. */
    void UpdateJointTorqueMax(void);

public:
    RobotInternal(const std::string & name, const mtsTaskPeriodic & owner, size_t numActuators, size_t numJoints);
    ~RobotInternal();

    void Configure(cmnXMLPath &xmlConfigFile, int robotNumber, AmpIO **BoardList);

    void SetupStateTable(mtsStateTable & stateTable);
    void SetupInterfaces(mtsInterfaceProvided * robotInterface,
                         mtsInterfaceProvided * actuatorInterface,
                         mtsStateTable & stateTable);

    bool CheckIfValid(void);
    inline bool IsValid(void) const { return this->Valid; }

    void GetData(void);
    void ConvertRawToSI(void);

    // List of functions for mtsRobotIO1394 connection manager.
    void GetName(std::string & placeHolder);
    void GetNumberOfActuators(int & placeHolder) const;
};

#endif  // _RobotInternal_h_
