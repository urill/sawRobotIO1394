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
#include <cisstCommon/cmnCommandLineOptions.h>
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

    cmnCommandLineOptions options;
    int port;
    std::string configFile;
    options.AddOptionOneValue("c", "config",
                              "configuration file, can be an absolute path or relative to CISST_ROOT share",
                              cmnCommandLineOptions::REQUIRED, &configFile);
    options.AddOptionOneValue("p", "port",
                              "firefire port number(s)",
                              cmnCommandLineOptions::REQUIRED, &port);
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    std::string fullFileName;
    if (cmnPath::Exists(configFile)) {
        fullFileName = configFile;
    } else {
        cmnPath path;
        path.AddRelativeToCisstShare("sawRobotIO1394");
        fullFileName = path.Find(configFile);
        if (fullFileName == "") {
            return 0;
        }
    }
    std::cout << "Configuration file: " << fullFileName << std::endl
              << "Port: " << port << std::endl;

    mtsManagerLocal *LCM = mtsManagerLocal::GetInstance();

    displayTask *disp = new displayTask("disp");
    mtsRobotIO1394 *robot = new mtsRobotIO1394("robot", 0.01, port, disp->GetOutputStream());

    // add the tasks to the component manager
    LCM->AddComponent(disp);
    LCM->AddComponent(robot);
    
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
    osaSleep(1.0 * cmn_s);

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
