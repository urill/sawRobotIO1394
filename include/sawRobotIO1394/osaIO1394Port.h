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

#ifndef _osaIO1394Port_h
#define _osaIO1394Port_h

#include <vector>
#include <map>

#ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <cisstCommon/cmnXMLPath.h>
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstParameterTypes/prmJointType.h>
#else
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "EigenWrapper.h"
#include "MinimalPrm.h"
#endif

#include <sawRobotIO1394/osaRobotIO1394.h>
#include <sawRobotIO1394/osaIO1394Robot.h>
#include <sawRobotIO1394/osaIO1394DigitalInput.h>

class AmpIO;
class FirewirePort;

namespace sawRobotIO1394 {
    class osaIO1394Port {
        /**
         * IO1394 Port Abstraction Layer
         * This class handles allocation, interfacing, and power-control for the QLA
         * robot control architecture. It is also responsible for some low-level error
         * handling.
         */

    public:

        osaIO1394Port(int portNumber, std::ostream & messageStream = std::cerr);
        ~osaIO1394Port();

        //! Add a robot to this port
        void Configure(const osaIO1394::Configuration & config);

        void AddRobot(const osaIO1394::RobotConfiguration & config);
        void AddRobot(osaIO1394Robot * Robot);

        void AddDigitalInput(const osaIO1394::DigitalInputConfiguration & config);
        void AddDigitalInput(osaIO1394DigitalInput * digital_input);

        //! Robot Accessors
        osaIO1394Robot * Robot(const std::string & name);
        const osaIO1394Robot * Robot(const std::string & name) const;

        osaIO1394Robot * Robot(const int index);
        const osaIO1394Robot * Robot(const int index) const;

        void GetRobotNames(std::vector<std::string> & names) const;
        void GetDigitalInputNames(std::vector<std::string> & names) const;

        //! Input/Ouput
        void Read(void);
        void Write(void);

        int NumberOfBoards(void) const;
        int NumberOfRobots(void) const;
        int NumberOfDigitalInputs(void) const;

    protected:

        //! Board Objects
        FirewirePort * Port_;

        std::map<int, AmpIO*> Boards_;
        typedef std::map<int, AmpIO*>::iterator board_iterator;
        typedef std::map<int, AmpIO*>::const_iterator board_const_iterator;

        //! Robot Objects
        std::vector<osaIO1394Robot*> Robots_;
        std::map<std::string, osaIO1394Robot*> RobotsByName_;
        typedef std::vector<osaIO1394Robot*>::iterator robot_iterator;
        typedef std::vector<osaIO1394Robot*>::const_iterator robot_const_iterator;

        std::vector<osaIO1394DigitalInput*> DigitalInputs_;
        std::map<std::string, osaIO1394DigitalInput*> DigitalInputsByName_;
        typedef std::vector<osaIO1394DigitalInput*>::iterator digital_input_iterator;
        typedef std::vector<osaIO1394DigitalInput*>::const_iterator digital_input_const_iterator;
    };
}

#endif // _osaIO1394Port_h
