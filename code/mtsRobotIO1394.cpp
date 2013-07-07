/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 $Id$

 Author(s):  Zihan Chen, Peter Kazanzides
 Created on: 2012-07-31

 (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <iostream>

#include <cisstCommon/cmnXMLPath.h>

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnUnits.h>

#include <cisstOSAbstraction/osaCPUAffinity.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>

#include <sawRobotIO1394/osaRobotIO1394.h>
#include <sawRobotIO1394/osaIO1394Port.h>
#include <sawRobotIO1394/osaIO1394Robot.h>
#include <sawRobotIO1394/osaIO1394XMLConfig.h>

#include "mtsIO1394Robot.h"
#include "mtsIO1394DigitalInput.h"

#include <FirewirePort.h>
#include <AmpIO.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsRobotIO1394, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg)

using namespace sawRobotIO1394;


mtsRobotIO1394::mtsRobotIO1394(const std::string & name, double periodInSeconds, int portNumber):
    mtsTaskPeriodic(name, periodInSeconds)
{
    Init(portNumber);
}

mtsRobotIO1394::mtsRobotIO1394(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
    Init(0);
}

mtsRobotIO1394::~mtsRobotIO1394()
{
    // delete port and message stream
    delete Port_;
    delete MessageStream;
}

void mtsRobotIO1394::Init(int port_num)
{
    // construct port
    MessageStream = new std::ostream(this->GetLogMultiplexer());
    try {
        Port_ = new sawRobotIO1394::osaIO1394Port(port_num, *MessageStream);
    } catch (std::runtime_error &err) {
        CMN_LOG_CLASS_INIT_ERROR << err.what();
        abort();
    }

    mtsInterfaceProvided * mainInterface = AddInterfaceProvided("MainInterface");
    if (mainInterface) {
        mainInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfBoards, this, "GetNumberOfBoards");
        mainInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfRobots, this, "GetNumberOfRobots");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "Init: failed to create provided interface \"MainInterface\", method Init should be called only once."
                                 << std::endl;
    }

    //////////////////////////////////////////////////////////////////
    ////////// RobotIO1394QtManager Configure Connection//////////////
    //////////////////////////////////////////////////////////////////

    // At this stage, the robot interfaces and the digital input interfaces should be ready.
    // Add on Configuration provided interface with functionWrite with vector of strings.
    // Provide names of robot, names of digital inputs, and name of this member.

    // All previous interfaces are ready. Good start. Let's make a new provided interface.
    mtsInterfaceProvided * configurationInterface   = this->AddInterfaceProvided("Configuration");
    if (configurationInterface) {
        configurationInterface->AddCommandRead(&osaIO1394Port::GetRobotNames, Port_,
                                               "GetRobotNames");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfActuatorPerRobot, this,
                                               "GetNumActuators");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfRobots, this,
                                               "GetNumRobots");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfDigitalInputs, this,
                                               "GetNumDigitalInputs");
        configurationInterface->AddCommandRead(&osaIO1394Port::GetDigitalInputNames, Port_,
                                               "GetDigitalInputNames");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetName, this,
                                               "GetName");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to create configurationInterface." << std::endl;
    }
}

void mtsRobotIO1394::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring from " << filename << std::endl;

    osaIO1394::Configuration config;
    osaIO1394XMLConfig::LoadFromFile(filename, config);

    // Add all the robots
    for (std::vector<osaIO1394::RobotConfiguration>::const_iterator it = config.Robots.begin();
         it != config.Robots.end();
         ++it) {
        // Create a new robot
        mtsIO1394Robot * robot = new mtsIO1394Robot(*this, *it);
        // Set up the cisstMultiTask interfaces
        if (!this->SetupRobot(robot)) {
            delete robot;
        } else {
            Robots_.push_back(robot);
        }
    }

    // Add all the digital inputs
    for (std::vector<osaIO1394::DigitalInputConfiguration>::const_iterator it = config.DigitalInputs.begin();
         it != config.DigitalInputs.end();
         ++it) {
        // Create a new robot
        mtsIO1394DigitalInput * digital_input = new mtsIO1394DigitalInput(*this, *it);
        // Set up the cisstMultiTask interfaces
        if (!this->SetupDigitalInput(digital_input)) {
            delete digital_input;
        } else {
            DigitalInputs_.push_back(digital_input);
        }
    }
}

bool mtsRobotIO1394::SetupRobot(mtsIO1394Robot * robot)
{
    mtsStateTable * stateTableRead;
    mtsStateTable * stateTableWrite;

    // Configure StateTable for this Robot
    if (!robot->SetupStateTables(this->StateTable.GetHistoryLength(),
                                 stateTableRead, stateTableWrite)) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupRobot: unable to setup state tables" << std::endl;
        return false;
    }

    this->AddStateTable(stateTableRead);
    this->AddStateTable(stateTableWrite);

    // Add new InterfaceProvided for this Robot with Name.
    // Ensure all names from XML Config file are UNIQUE!
    mtsInterfaceProvided * robotInterface = this->AddInterfaceProvided(robot->Name());
    if (!robotInterface) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupRobot: failed to create robot interface \""
                                 << robot->Name() << "\", do we have multiple robots with the same name?" << std::endl;
        return false;
    }

    // Create actuator interface
    std::string actuatorInterfaceName = robot->Name();
    actuatorInterfaceName.append("Actuators");
    mtsInterfaceProvided * actuatorInterface = this->AddInterfaceProvided(actuatorInterfaceName);
    if (!actuatorInterface) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupRobot: failed to create robot actuator interface \""
                                 << actuatorInterfaceName << "\", do we have multiple robots with the same name?" << std::endl;
        return false;
    }

    // Setup the MTS interfaces
    robot->SetupInterfaces(robotInterface, actuatorInterface);

    // Add the robot to the port
    try {
        Port_->AddRobot(robot);
    } catch (osaIO1394::configuration_error & err) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupRobot: unable to add the robot to the port: " << err.what() << std::endl;
        return false;
    }
    return true;
}

bool mtsRobotIO1394::SetupDigitalInput(mtsIO1394DigitalInput * digitalInput)
{
    // Configure pressed active direction and edge detection
    digitalInput->SetupStateTable(this->StateTable);

    mtsInterfaceProvided * digitalInInterface = this->AddInterfaceProvided(digitalInput->Name());

    digitalInput->SetupProvidedInterface(digitalInInterface, this->StateTable);

    // Add the mechanism to the port
    try {
        Port_->AddDigitalInput(digitalInput);
    } catch (osaIO1394::configuration_error & err) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupDigitalInput: unable to add the robot to the port: " << err.what() << std::endl;
        return false;
    }
    return true;
}

void mtsRobotIO1394::Startup(void)
{
    // osaCPUSetAffinity(OSA_CPU4);
}

void mtsRobotIO1394::PreRead(void)
{
    const robots_iterator robots_end = Robots_.end();
    for (robots_iterator robot = Robots_.begin();
         robot != robots_end;
         ++robot) {
        (*robot)->StartReadStateTable();
    }
}

void mtsRobotIO1394::PostRead(void)
{
    // Trigger robot events
    const robots_iterator robots_end = Robots_.end();
    for (robots_iterator robot = Robots_.begin();
         robot != robots_end;
         ++robot) {
        (*robot)->CheckState();
        (*robot)->AdvanceReadStateTable();
    }
    // Trigger digital input events
    const digital_inputs_iterator digital_inputs_end = DigitalInputs_.end();
    for (digital_inputs_iterator digital_input = DigitalInputs_.begin();
         digital_input != digital_inputs_end;
         ++digital_input) {
        (*digital_input)->CheckState();
    }
}

void mtsRobotIO1394::Run(void)
{
    // Read from all boards
    this->PreRead();
    Port_->Read();
    this->PostRead();

    // Invoke connected components (if any)
    this->RunEvent();

    // Process queued commands (e.g., to set motor current)
    this->ProcessQueuedCommands();

    // Write to all boards
    Port_->Write();
}

void mtsRobotIO1394::Cleanup(void)
{
    for (size_t i = 0; i < Robots_.size(); i++) {
        if (Robots_[i]->Valid()) {
            Robots_[i]->DisablePower();
        }
    }
    // Write to all boards
    Port_->Write();
}

void mtsRobotIO1394::GetNumberOfDigitalInputs(int & placeHolder) const
{
    placeHolder = Port_->NumberOfDigitalInputs();
}

void mtsRobotIO1394::GetNumberOfBoards(int & placeHolder) const
{
    placeHolder = Port_->NumberOfBoards();
}

void mtsRobotIO1394::GetNumberOfRobots(int & placeHolder) const
{
    placeHolder = Port_->NumberOfRobots();
}

void mtsRobotIO1394::GetNumberOfActuatorPerRobot(vctIntVec & placeHolder) const
{
    size_t num_robots = Port_->NumberOfRobots();
    placeHolder.resize(num_robots);

    for (size_t i = 0; i < num_robots; i++) {
        placeHolder[i] = Port_->Robot(i)->NumberOfActuators();
    }
}

void mtsRobotIO1394::GetName(std::string & placeHolder) const
{
    placeHolder = mtsComponent::GetName();
}
