/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Jonathan Bohren
  Created on: 2013-06-29

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaIO1394XMLConfig_h
#define _osaIO1394XMLConfig_h

#include <cisstCommon/cmnXMLPath.h>

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnUnits.h>

#include <cisstParameterTypes/prmJointType.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

#include <sawRobotIO1394/osaRobotIO1394.h>

namespace sawRobotIO1394 {
    class osaIO1394XMLConfig {
    public:
        static void LoadFromFile(const std::string & filename,
                                 osaIO1394::Configuration & config);

        static bool ConfigureRobot(cmnXMLPath & xmlConfig,
                                   const int robotIndex,
                                   osaIO1394::RobotConfiguration & robot);

        static bool ConfigureCoupling(cmnXMLPath & xmlConfig,
                                      const int robotIndex,
                                      osaIO1394::RobotConfiguration & robot);

        static bool ConfigureCouplingMatrix(cmnXMLPath & xmlConfig,
                                            const int robotIndex,
                                            const char * couplingString,
                                            int numRows,
                                            int numCols,
                                            vctDoubleMat & resultMatrix);

        static bool ConfigureDigitalInput(cmnXMLPath & xmlConfig,
                                          const int tagIndex,
                                          osaIO1394::DigitalInputConfiguration & digitalInput);
    };
}

#endif // _osaIO1394XMLConfig_h
