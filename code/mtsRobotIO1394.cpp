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


#include "FirewirePort.h"
#include "AmpIO.h"
#include "RobotInternal.h"
#include "DigitalInInternal.h"


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsRobotIO1394, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg)


//============ mtsRobotIO1394 =========================================

mtsRobotIO1394::mtsRobotIO1394(const std::string &name, double period, int port_num,
                               std::ostream &debugStream) : mtsTaskPeriodic(name, period)
{
    Init();
    Port = new FirewirePort(port_num, debugStream);
}

mtsRobotIO1394::mtsRobotIO1394(const mtsTaskPeriodicConstructorArg &arg)
    : mtsTaskPeriodic(arg)
{
    Init();
    Port = new FirewirePort(0);
}

mtsRobotIO1394::~mtsRobotIO1394()
{
    // Delete boards
    for(int i = 0; i < mtsRobotIO1394::MAX_BOARDS; i++) {
        if (BoardList[i]) {
            Port->RemoveBoard(i);
            delete BoardList[i];
        }
    }
    delete Port;
}

void mtsRobotIO1394::Init(void)
{
    for (int i = 0; i < mtsRobotIO1394::MAX_BOARDS; i++)
        BoardList[i] = 0;

    mtsInterfaceProvided * mainInterface = AddInterfaceProvided("MainInterface");
    if (mainInterface) {
        mainInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfBoards, this, "GetNumberOfBoards");
        mainInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfRobots, this, "GetNumberOfRobots");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "Init: failed to create provided interface \"MainInterface\", method Init should be called only once."
                                 << std::endl;
    }
}

void mtsRobotIO1394::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configuring from " << filename << std::endl;

    cmnXMLPath xmlConfig;
    xmlConfig.SetInputSource(filename);
    char path[64];

    //std::cout<<"The number of robots detected are " << RobotCounter << std::endl;

    int tmpNumActuator = 0;
    int tmpBoardID = 0;
    int tmpAxisID = 0;

    std::string context = "Config";
    std::string tmpRobotName = "ConfigStart";

    //Let's activate the boards implied in the XML config file.
    //Go robot by robot, Actuator by Actuator, and look for BoardID.
    //This will not check for conflicting BoardID/AxisID assignments.
    //Infinite loop for configuring for each robot.
    int k = 0;

    while (true) {
        //Incrementing Counter for Robot.
        k = k + 1;
        sprintf(path, "Robot[%d]/@Name",k);
        xmlConfig.GetXMLValue(context.c_str(), path, tmpRobotName);
        if (tmpRobotName.empty()) {
            //Reached the End of File For Robot
            //Break from infinite loop.
            //std::cout<<"Run Number "<< k <<" stopped." << std::endl;
            //std::cin.ignore();
            break;
        }
        sprintf(path, "Robot[%d]/@NumOfActuator", k);
        xmlConfig.GetXMLValue(context.c_str(), path, tmpNumActuator);

        //Create new temporary RobotInternal Initialized.
        RobotInternal * robot = new RobotInternal(tmpRobotName, *this, tmpNumActuator);

        //Set ActuatorInfo for all Actuators under this robot.
        int j = 0;
        for (j = 0; j < tmpNumActuator; j++) {
            sprintf(path,"Robot[%d]/Actuator[%d]/@BoardID",k,j+1);
            xmlConfig.GetXMLValue(context.c_str(), path, tmpBoardID);
            // Check and initalize AmpIOs for new boards.
            if(BoardList[tmpBoardID] == 0){
                BoardList[tmpBoardID] = new AmpIO(tmpBoardID);
                Port->AddBoard(BoardList[tmpBoardID]);
            }
            // Config this Actuator info.
            sprintf(path,"Robot[%d]/Actuator[%d]/@AxisID",k,j+1);
            xmlConfig.GetXMLValue(context.c_str(), path, tmpAxisID);
            robot->SetActuatorInfo(j, BoardList[tmpBoardID], tmpAxisID);
        }
        // Configure conversion factors and other variables.
        robot->Configure(xmlConfig, k);
        // Configure StateTable for this Robot
        robot->SetupStateTable(this->StateTable);

        // Add new InterfaceProvided for this Robot with Name.
        // Ensure all tmpRobotNames from XML Config file are UNIQUE!
        mtsInterfaceProvided * robotInterface = this->AddInterfaceProvided(tmpRobotName);
        if (!robotInterface) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to create robot interface \""
                                     << tmpRobotName << "\", do we have multiple robots with the same name?" << std::endl;
            delete robot;
            robot = 0;
        }
        std::string actuatorInterfaceName = tmpRobotName;
        actuatorInterfaceName.append("Actuators");
        mtsInterfaceProvided * actuatorInterface = this->AddInterfaceProvided(actuatorInterfaceName);
        if (!actuatorInterface) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to create robot actuator interface \""
                                     << actuatorInterfaceName << "\", do we have multiple robots with the same name?" << std::endl;
            delete robot;
            robot = 0;
        }
        if (robot) {
            robot->SetupInterfaces(robotInterface, actuatorInterface, this->StateTable);
        }

        // Store the robot to RobotList
        RobotList.push_back(robot);
    }

    //////////////////////////////////////////////////////////////////
    ///////////////// Digital Input Setup Stage///////////////////////
    //////////////////////////////////////////////////////////////////

    k = 0;
    //Setup Digital Input Lists.
    bool readStat0 = false;
    bool readStat1 = false;
    bool readStat2 = false;
    std::string tmpDIName = "";
    tmpBoardID = -1;
    int tmpBitID = -1;

    while (true) {
        //Initialize read confirm stats.
        readStat0 = false;
        readStat1 = false;
        readStat2 = false;

        //Check there is digital input entry. Return boolean result for success/fail.
        k = k + 1;
        sprintf(path,"DigitalIn[%i]/@Name", k);
        readStat0 = xmlConfig.GetXMLValue(context.c_str(),path,tmpDIName);
        sprintf(path,"DigitalIn[%i]/@BoardID", k);
        readStat1 = xmlConfig.GetXMLValue(context.c_str(),path,tmpBoardID);
        sprintf(path,"DigitalIn[%i]/@BitID", k);
        readStat2 = xmlConfig.GetXMLValue(context.c_str(),path,tmpBitID);

        if(!readStat0 || !readStat1 || !readStat2){
            CMN_LOG_INIT_ERROR<<"Configuration for "<<path<<" finished/failed. Stopping config."<<std::endl;
            CMN_LOG_INIT_ERROR<<"Total number of DigitalIn available: "<< DigitalInList.size() <<". "<<std::endl;
            break;
        }
        //If not broken, go ahead.
        DigitalInInternal * digitalIn = new DigitalInInternal(*this,tmpDIName,BoardList[tmpBoardID],tmpBitID);
        // Make new DigitalIn provided interface.
        digitalIn->Configure(xmlConfig,k);
        // Configure pressed active direction and edge detection
        digitalIn->SetupStateTable(this->StateTable);

        mtsInterfaceProvided * digitalInInterface = this->AddInterfaceProvided(tmpDIName);

        digitalIn->SetupProvidedInterface(digitalInInterface,this->StateTable);
        DigitalInList.push_back(digitalIn);
    }
}

void mtsRobotIO1394::Startup(void)
{
    // osaCPUSetAffinity(OSA_CPU4);
}

void mtsRobotIO1394::Run(void)
{
    size_t i;
    // Read from all boards
    Port->ReadAllBoards();
    // Loop through the robots, processing feedback
    for (i = 0; i < RobotList.size(); i++) {
        if (RobotList[i]->CheckIfValid()) {
            // Copy data to state table
            RobotList[i]->GetData();
            // Convert from raw to SI units (TBD)
            RobotList[i]->ConvertRawToSI();
        }
    }
    // Loop through the digital inputs
    for (i = 0; i < DigitalInList.size(); i++) {
        DigitalInList[i]->GetData();
    }
    // Invoke connected components (if any)
    RunEvent();
    // Process queued commands (e.g., to set motor current)
    ProcessQueuedCommands();
    // Write to all boards
    Port->WriteAllBoards();
}

void mtsRobotIO1394::Cleanup(void)
{
}

void mtsRobotIO1394::GetNumberOfDigitalInputs(int &num) const
{
    num = DigitalInList.size();
}

void mtsRobotIO1394::GetNumberOfBoards(int &num) const
{
    num = 0;
    for (size_t i = 0; i < mtsRobotIO1394::MAX_BOARDS; i++)
        if (BoardList[i]) num++;
}

void mtsRobotIO1394::GetNumberOfRobots(int &num) const
{
    num = RobotList.size();
}
