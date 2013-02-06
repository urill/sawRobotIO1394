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

#include <cisstCommon/cmnTokenizer.h>   // TEMP
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
    CMN_LOG_CLASS_INIT_VERBOSE << "Configuring from " << filename << std::endl;

    // For now, use filename to specify boards (and assume just one robot)
    // In the future, change this to be an XML file
    cmnTokenizer tokens;
    tokens.Parse(filename);

    unsigned int numBoards = tokens.GetNumTokens()-1;  // -1 for NULL at end
    unsigned int numJoints = 4*numBoards;
    RobotInternal *robot = new RobotInternal("Robot", numJoints);
    unsigned int i, j;
    for (i = 0, j = 0; i < numBoards; i++, j += 4) {
        int bd = atoi(tokens.GetToken(i));
        BoardList[bd] = new AmpIO(bd);
        Port->AddBoard(BoardList[bd]);
        robot->SetJointInfo(j, BoardList[bd], j);
        robot->SetJointInfo(j+1, BoardList[bd], j+1);
        robot->SetJointInfo(j+2, BoardList[bd], j+2);
        robot->SetJointInfo(j+3, BoardList[bd], j+3);
    }
    robot->SetupStateTable(StateTable);
    mtsInterfaceProvided* prov = AddInterfaceProvided("Robot");
    robot->SetupProvidedInterface(prov, StateTable);
    RobotList.push_back(robot);
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
