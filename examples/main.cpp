/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*
  $Id$

  Author(s):  Peter Kazanzides
  Created on: 2013-01-31

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include "displayTask.h"
#include <cisstCommon/cmnPath.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>

int main(int argc, char ** argv)
{
    // log configuration
    // get all messages to log file
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    // get only errors and warnings to cerr
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    int i;

    int port = 0;
    int args_found = 0;
    std::string boardList;
    for (i = 1; i < argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            port = atoi(argv[i]+2);
            std::cerr << "Selecting port " << port << std::endl;
        }
        else {
            boardList.append(argv[i]);
            boardList.append(" ");
            args_found++;
        }
    }
    if (args_found < 1) {
        std::cerr << "Usage: RobotDisplay <board0> [<board1> ...] [-pP]" << std::endl
                  << "       where P = port number (default 0)" << std::endl;
        return 0;
    }

    mtsManagerLocal *LCM = mtsManagerLocal::GetInstance();

    displayTask *disp = new displayTask("disp");
    mtsRobotIO1394 *robot = new mtsRobotIO1394("robot", 0.01, port, disp->GetOutputStream());

    // add the tasks to the component manager
    LCM->AddComponent(disp);
    LCM->AddComponent(robot);
    
    cmnPath path;
    path.AddRelativeToCisstShare("sawRobotIO1394");
    const std::string fileName = "sawRobotIO1394TestBoard.xml";
    std::string fullFileName = path.Find(fileName);
    if (fullFileName == "") {
        return 0;
    }
    robot->Configure(fullFileName);
    disp->Configure();

    LCM->Connect("disp", "Robot", "robot", "Robot");

    // create the components
    LCM->CreateAll();
    LCM->WaitForStateAll(mtsComponentState::READY, 2.0 * cmn_s);

    // start the periodic Run
    LCM->StartAll();

    // On exit, wait for displayTask to be finished before killing
    // other tasks (such as mtsRobotIO1394), so that displayTask::Cleanup
    // can still invoke commands in mtsRobotIO1394.
    // For now, we just sleep for 1 second.
    osaSleep(1.0);

    // cleanup
    LCM->KillAll();
    LCM->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    LCM->Cleanup();

    delete disp;
    delete robot;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
