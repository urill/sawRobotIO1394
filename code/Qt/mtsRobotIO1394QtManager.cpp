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


// system include
#include <iostream>

// project include
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include "sawRobotIO1394/mtsRobotIO1394QtManager.h"

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsRobotIO1394QtManager, mtsComponent, std::string);


mtsRobotIO1394QtManager::mtsRobotIO1394QtManager(const std::string &name):
    mtsComponent(name),
    BuildWidgetsCalled(false)
{
    // This function will make the required interface to be connected with
    // the provided interface of mtsRobotIO1394 named Configure with predefined function names.
    RobotConfigureInterface = AddInterfaceRequired("Configuration_Qt");
    if (RobotConfigureInterface) {
        RobotConfigureInterface->AddFunction("GetRobotNames",Configuration.GetRobotNames);
        RobotConfigureInterface->AddFunction("GetNumActuators",Configuration.GetNumbersOfActuators);

        RobotConfigureInterface->AddFunction("GetNumRobots",Configuration.GetNumberOfRobots);
        RobotConfigureInterface->AddFunction("GetNumDigitalInputs",Configuration.GetNumberOfDigitalInputs);

        RobotConfigureInterface->AddFunction("GetDigitalInputNames",Configuration.GetDigitalInputNames);
        RobotConfigureInterface->AddFunction("GetName",Configuration.GetName);
    }
}

void mtsRobotIO1394QtManager::Configure(const std::string &) {
    this->BuildWidgets();
}

void mtsRobotIO1394QtManager::Startup(void) {
        this->BuildWidgets();
}


void mtsRobotIO1394QtManager::BuildWidgets(void)
{
    if (BuildWidgetsCalled) return;
    BuildWidgetsCalled = true;

    if (!RobotConfigureInterface->GetConnectedInterface()) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: unable to connect to configuration interface"
                                 << std::endl;
        return;
    }

    size_t i = 0;

    std::string qtSuffix = "QtWidget";
    std::string actuatorInterfaceSuffix = "Actuators";
    std::string newComponentName;
    std::string newInterfaceActuatorName;

    std::string tmpRobotName;

    Configuration.GetName(NameOfRobotIO1394);
    Configuration.GetNumberOfRobots(NumberOfRobots);

    if (NumberOfRobots > 0) {
        RobotNames.resize(NumberOfRobots);
        NumberOfActuatorsPerRobot.resize(NumberOfRobots);
    }
    else {
        CMN_LOG_CLASS_INIT_ERROR << "BuildWidgets: no robot found" << std::endl;
    }

    mtsManagerLocal * LCM = mtsManagerLocal::GetInstance();

    Configuration.GetRobotNames(RobotNames);
    Configuration.GetNumbersOfActuators(NumberOfActuatorsPerRobot);

    for (i = 0; i < NumberOfRobots; i++) {
        tmpRobotName = RobotNames[i];
        newComponentName = tmpRobotName.append(qtSuffix);
        tmpRobotName = RobotNames[i];
        newInterfaceActuatorName = tmpRobotName.append(actuatorInterfaceSuffix);
        tmpRobotName = RobotNames[i];

        mtsRobotIO1394QtWidget *robotDisplay =
                new mtsRobotIO1394QtWidget(newComponentName, NumberOfActuatorsPerRobot[i]);
        robotDisplay->Configure();
        LCM->AddComponent(robotDisplay);
        LCM->Connect(newComponentName, "Robot", NameOfRobotIO1394, tmpRobotName);
        LCM->Connect(newComponentName, "RobotActuators", NameOfRobotIO1394, newInterfaceActuatorName);

        robotDisplay->Create();
        robotDisplay->Start();
    }
}
