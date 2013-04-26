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


mtsRobotIO1394QtManager::mtsRobotIO1394QtManager(const std::string &name) : mtsComponent(name)
{
    // This function will make the required interface to be connected with
    // the provided interface of mtsRobotIO1394 named Configure with predefined function names.
    robotConfigureInterface = AddInterfaceRequired("Configuration_Qt");
    if (robotConfigureInterface) {
        robotConfigureInterface->AddFunction("GetRobotNames",Configuration.getRobotNames_Qt);
        robotConfigureInterface->AddFunction("GetNumActuators",Configuration.getNumActuators_Qt);

        robotConfigureInterface->AddFunction("GetNumRobots",Configuration.getNumRobots_Qt);
        robotConfigureInterface->AddFunction("GetNumDigitalInputs",Configuration.getNumDigital_Qt);

        robotConfigureInterface->AddFunction("GetDigitalInputNames",Configuration.getDigitalInputNames_Qt);
        robotConfigureInterface->AddFunction("GetName",Configuration.getName_Qt);
    }
}

void mtsRobotIO1394QtManager::Configure(void) {
    if (robotConfigureInterface->GetConnectedInterface()) {
        this->BuildWidgets();
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to connect to configuration interface"
                                 << std::endl;
    }
}

void mtsRobotIO1394QtManager::Startup(void) {
}


void mtsRobotIO1394QtManager::BuildWidgets(void) {
    int i = 0;

    std::string qt_Addon = "_Qt";
    std::string actuator_Addon = "Actuators";
    std::string qt_Added;
    std::string actuator_Added;

    std::string tmpRobotName;

    Configuration.getName_Qt(nameOfRobotIO1394);
    Configuration.getNumRobots_Qt(numberOfRobots);

    if(numberOfRobots > 0) {
        nameOfRobots.resize(numberOfRobots);
        numberOfActuatorsPerRobot.resize(numberOfRobots);
    }
    else {
        CMN_LOG_CLASS_INIT_ERROR << "buildWidgets: Number of robots less than 1" << std::endl;
    }

    mtsManagerLocal *LCM = mtsManagerLocal::GetInstance();

    Configuration.getRobotNames_Qt(nameOfRobots);
    Configuration.getNumActuators_Qt(numberOfActuatorsPerRobot);
    Configuration.getNumDigital_Qt(numberOfDigitalInputs);

    for(i = 0; i < numberOfRobots; i++) {
        tmpRobotName = nameOfRobots[i];
        qt_Added = tmpRobotName.append(qt_Addon);
        tmpRobotName = nameOfRobots[i];
        actuator_Added = tmpRobotName.append(actuator_Addon);
        tmpRobotName = nameOfRobots[i];

        mtsRobotIO1394QtWidget *robotDisplay =
                new mtsRobotIO1394QtWidget(qt_Added,numberOfActuatorsPerRobot[i]);
        robotDisplay->Configure();
        LCM->AddComponent(robotDisplay);
        LCM->Connect(qt_Added, "Robot", nameOfRobotIO1394, tmpRobotName);
        LCM->Connect(qt_Added, "RobotActuators", nameOfRobotIO1394, actuator_Added);

        robotDisplay->Create();
        robotDisplay->Start();
        robotDisplay->show();
    }
}
