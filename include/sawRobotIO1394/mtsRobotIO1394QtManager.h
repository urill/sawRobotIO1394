/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Kwang Young Lee
  Created on: 2013-04-11

  (C) Copyright 2012-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef mtsRobotIO1394QtManager_h
#define mtsRobotIO1394QtManager_h

#include <cisstCommonXML.h>
#include <cisstOSAbstraction/osaTimeServer.h>
#include <cisstVector/vctQtWidgetDynamicVector.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsIntervalStatistics.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>

#include <QtCore>
#include <QtGui>

#include <sawRobotIO1394/mtsRobotIO1394QtWidget.h>

class mtsRobotIO1394QtManager: public mtsComponent
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsRobotIO1394QtManager(const std::string &name);
    inline ~mtsRobotIO1394QtManager(void) {}

    void Configure(const std::string &filename);
    void Startup(void);

    void BuildWidgets(void);

protected:
    mtsInterfaceRequired * robotConfigureInterface;
    std::string nameOfRobotIO1394;
    std::vector<std::string> nameOfRobots;
    std::vector<std::string> nameOfDigitalInputs;

    int numberOfRobots;
    int numberOfDigitalInputs;

    vctIntVec numberOfActuatorsPerRobot;

    struct ConfigStruct {
        mtsFunctionRead getNumRobots_Qt;
        mtsFunctionRead getNumDigital_Qt;

        mtsFunctionRead getRobotNames_Qt;
        mtsFunctionRead getNumActuators_Qt;

        mtsFunctionRead getDigitalInputNames_Qt;
        mtsFunctionRead getName_Qt;
    } Configuration;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobotIO1394QtManager);

#endif // mtsRobotIO1394QtManager_h
