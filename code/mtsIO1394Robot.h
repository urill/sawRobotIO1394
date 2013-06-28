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

#ifndef _mtsIO1394Robot_h
#define _mtsIO1394Robot_h

#include <sawRobotIO1394/mtsRobotIO1394.h>

#include <cisstParameterTypes/prmJointType.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

#include <sawRobotIO1394/osaRobotIO1394.h>
#include <sawRobotIO1394/osaIO1394Robot.h>

namespace sawRobotIO1394 {

    class mtsIO1394Robot: public osaIO1394Robot {
    public:
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

        mtsIO1394Robot(const cmnGenericObject & owner,
                       const osaIO1394::RobotConfiguration & config);

        void SetupStateTable(mtsStateTable & stateTable);
        void SetupInterfaces(mtsInterfaceProvided * robotInterface,
                             mtsInterfaceProvided * actuatorInterface,
                             mtsStateTable & stateTable);

        void TriggerEvents(void);

        // Wrapper of osa methods to match command signatures
        void GetNumberOfActuators(int & num_actuators) const;
        void GetNumberOfJoints(int & num_joints) const;
        void SetTorqueJoint(const prmForceTorqueJointSet & jointTorques);
        void EnableSafetyRelay(void);
        void DisableSafetyRelay(void);
        void ResetSingleEncoder(const int & index);

    protected:
        prmForceTorqueJointSet TorqueJoint;
        prmPositionJointGet PositionJointGet;
        prmPositionJointGet PositionActuatorGet;

        // Functions for events
        struct {
            mtsFunctionWrite PowerStatus;
        } EventTriggers;
    };

}

#endif // _mtsIO1394Robot_h
