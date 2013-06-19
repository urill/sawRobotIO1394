/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Kwang Young Lee
  Created on: 2013-04-11

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsRobotIO1394QtManager_h
#define _mtsRobotIO1394QtManager_h

#include <cisstCommonXML.h>
#include <cisstOSAbstraction/osaTimeServer.h>
#include <cisstVector/vctQtWidgetDynamicVector.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsIntervalStatistics.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>

#include <QtCore>
#include <QtGui>

#include <sawRobotIO1394/mtsRobotIO1394QtWidget.h>

class mtsRobotIO1394QtWidgetFactory: public mtsComponent
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsRobotIO1394QtWidgetFactory(const std::string & name);
    inline ~mtsRobotIO1394QtWidgetFactory(void) {}

    void Configure(const std::string & filename = "");
    void Startup(void);

protected:
    void BuildWidgets(void);
    bool BuildWidgetsCalled;
    mtsInterfaceRequired * RobotConfigureInterface;
    std::string NameOfRobotIO1394;
    std::vector<std::string> RobotNames;
    std::vector<std::string> DigitalInputNames;

    int NumberOfRobots;
    int NumberOfDigitalInputs;

    vctIntVec NumberOfActuatorsPerRobot;

    struct ConfigStruct {
        mtsFunctionRead GetNumberOfRobots;
        mtsFunctionRead GetNumberOfDigitalInputs;

        mtsFunctionRead GetRobotNames;
        mtsFunctionRead GetNumbersOfActuators;

        mtsFunctionRead GetDigitalInputNames;
        mtsFunctionRead GetName;
    } Configuration;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobotIO1394QtWidgetFactory);

#endif // _mtsRobotIO1394QtManager_h
