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

#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>


#include "FirewirePort.h"
#include "AmpIO.h"
#include "RobotInternal.h"


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

    mtsInterfaceProvided* prov = AddInterfaceProvided("MainInterface");
    if (prov) {
        prov->AddCommandRead(&mtsRobotIO1394::GetNumberOfBoards, this, "GetNumberOfBoards");
        prov->AddCommandRead(&mtsRobotIO1394::GetNumberOfRobots, this, "GetNumberOfRobots");
    }
}

void mtsRobotIO1394::Configure(const std::string &filename)
{
    //CMN_LOG_CLASS_INIT_VERBOSE << "Configuring from " << filename << std::endl;

    cmnPath localPath;

    localPath.Add("config");
    std::string localFile = localPath.Find(filename);

    cmnXMLPath xmlConfig;

    xmlConfig.SetInputSource(localFile);
    char path[64];

    //std::cout<<"The number of robots detected are " << RobotCounter << std::endl;

    int tmpNumActuator=0;
    int tmpBoardID=0;
    int tmpAxisID=0;

    std::string context = "Config";

    std::string tmpRobotName = "ConfigStart";


    //Let's activate the boards implied in the XML config file.
    //Go robot by robot, Actuator by Actuator, and look for BoardID.
    //This will not check for conflicting BoardID/AxisID assignments.
    //Infinite loop for configuring for each robot.
    int k=0;

    while(true){
        //Incrementing Counter for Robot.
        k = k + 1;
        std::cout<<"This is Run Number " << k <<"."<< std::endl;

        sprintf(path, "Robot[%d]/@Name",k);
        xmlConfig.GetXMLValue(context.c_str(),path,tmpRobotName);
        if(tmpRobotName.empty()){
            //Reached the End of File For Robot
            //Break from infinite loop.
            //std::cout<<"Run Number "<< k <<" stopped." << std::endl;
            //std::cin.ignore();
            break;
        }
        sprintf(path, "Robot[%d]/@NumOfActuator",k);
        xmlConfig.GetXMLValue(context.c_str(),path,tmpNumActuator);

        //Create new temporary RobotInternal Initialized.
        RobotInternal *robot = new RobotInternal(tmpRobotName,tmpNumActuator);

        //Set ActuatorInfo for all Actuators under this robot.
        int j = 0;
        for(j = 0; j < tmpNumActuator; j++) {
            sprintf(path,"Robot[%d]/Actuator[%d]/@BoardID",k,j+1);
            xmlConfig.GetXMLValue(context.c_str(),path,tmpBoardID);
            //Check and initalize AmpIOs for new boards.
            if(BoardList[tmpBoardID] == 0){
                BoardList[tmpBoardID] = new AmpIO(tmpBoardID);
                Port->AddBoard(BoardList[tmpBoardID]);
            }
            //Config this Actuator info.
            sprintf(path,"Robot[%d]/Actuator[%d]/@AxisID",k,j+1);
            xmlConfig.GetXMLValue(context.c_str(),path,tmpAxisID);
            robot->SetJointInfo(j,BoardList[tmpBoardID],tmpAxisID);
        }
        //Configure conversion factors and other variables.
        robot->Configure(xmlConfig, k);
        //Configure StateTable for this Robot
        robot->SetupStateTable(StateTable);

        //Add new InterfaceProvided for this Robot with Name.
        //Ensure all tmpRobotNames from XML Config file are UNIQUE!
        mtsInterfaceProvided* prov=AddInterfaceProvided(tmpRobotName);
        robot->SetupProvidedInterface(prov,StateTable);

        //Store the robot to RobotList
        RobotList.push_back(robot);

    }
}

void mtsRobotIO1394::Startup(void)
{
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
