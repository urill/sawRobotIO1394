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

class mtsRobotIO1394::RobotInternal {
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
    int NumberOfActuators;
    int NumberOfJoints;
    bool HasActuatorToJointCoupling;

    // State data
    bool           Valid;
    bool           PowerStatus;
    unsigned short SafetyRelay;
    unsigned long  watchdogPeriod;
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
    vctULongVec    digitalIn;
    vctLongVec     motorFeedbackCurrentRaw;
    vctDoubleVec   motorFeedbackCurrent;
    prmForceTorqueJointSet TorqueJoint;
    vctLongVec     motorControlCurrentRaw;
    vctDoubleVec   motorControlCurrent;
    vctDoubleVec   motorControlTorque;
    vctIntVec      encSetPosRaw;
    vctDoubleVec   encSetPos;

    vctDoubleMat ActuatorToJointPosition;
    vctDoubleMat JointToActuatorPosition;
    vctDoubleMat ActuatorToJointTorque;
    vctDoubleMat JointToActuatorTorque;

    // Methods for provided interface
    void GetNumberOfActuators(int & placeHolder) const;
    void GetNumberOfJoints(int & placeHolder) const;
    void EnablePower(void);
    void DisablePower(void);
    void EnableSafetyRelay(void);
    void DisableSafetyRelay(void);
    void SetWatchdogPeriod(const unsigned long &period_ms);
    void SetAmpEnable(const vctBoolVec &ampControl);
    void SetTorqueJoint(const prmForceTorqueJointSet & torques);
    void SetMotorCurrentRaw(const vctLongVec &mcur);
    void SetMotorCurrent(const vctDoubleVec &mcur);
    void ResetAmpsToBitsOffsetUsingFeedbackAmps(void);
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

public:

    RobotInternal(const std::string & name, const cmnGenericObject & owner, size_t numActuators);
    ~RobotInternal();
    void Configure(const std::string &filename);
    void Configure (cmnXMLPath &xmlConfigFile, int robotNumber);
    void ConfigureCoupling (cmnXMLPath & xmlConfigFile, int robotNumber);
    void ConfigureCouplingA2J(cmnXMLPath &xmlConfigFile, int robotNumber, int numOfActuator,
                              int numOfJoint, vctDoubleMat &A2JMatrix);
    void ConfigureCouplingJ2A(cmnXMLPath &xmlConfigFile, int robotNumber, int numOfActuator,
                              int numOfJoint, vctDoubleMat &J2AMatrix);
    void ConfigureCouplingAT2JT(cmnXMLPath &xmlConfigFile, int robotNumber, int numOfActuator,
                                int numOfJoint, vctDoubleMat &AT2JTMatrix);
    void ConfigureCouplingJT2AT(cmnXMLPath &xmlConfigFile, int robotNumber, int numOfActuator,
                                int numOfJoint, vctDoubleMat &JT2ATMatrix);
    void ConfigureCouplingMatrix(cmnXMLPath &xmlConfigFile, const std::string pathToMatrix, int numRows,
                                 int numCols, vctDoubleMat &resultMatrix);

    void SetActuatorInfo(int index, AmpIO *board, int axis);

    void SetupStateTable(mtsStateTable & stateTable);
    void SetupInterfaces(mtsInterfaceProvided * robotInterface,
                         mtsInterfaceProvided * actuatorInterface,
                         mtsStateTable & stateTable);

    bool CheckIfValid(void);
    inline bool IsValid(void) const { return this->Valid; }

    void GetData(void);
    void ConvertRawToSI(void);
};

#endif  // _RobotInternal_h_
