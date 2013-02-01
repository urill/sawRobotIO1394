/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: displayTask.cpp 3797 2012-08-21 04:02:56Z pkazanz1 $

  (C) Copyright 2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <curses.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include "displayTask.h"

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(displayTask, mtsTaskContinuous, std::string)

// passing false to mtsTaskContinuous causes it to use the main thread
displayTask::displayTask(const std::string & taskName) : mtsTaskContinuous(taskName, 256, false),
    debugStream(std::stringstream::in), last_debug_line(DEBUG_START_LINE)
{
    mtsInterfaceRequired *req = AddInterfaceRequired("Robot");
    if (req) {
        req->AddFunction("GetNumberOfJoints", Robot.GetNumberOfJoints);
        req->AddFunction("IsValid", Robot.IsValid);
        req->AddFunction("GetPositionRaw", Robot.GetPositionRaw);
        req->AddFunction("GetVelocityRaw", Robot.GetVelocityRaw);
        req->AddFunction("GetAnalogInRaw", Robot.GetAnalogInRaw);
        req->AddFunction("GetMotorCurrentRaw", Robot.GetMotorCurrentRaw);
        req->AddFunction("SetMotorCurrentRaw", Robot.SetMotorCurrentRaw);
    }
}

displayTask::~displayTask()
{
}

void displayTask::Configure(const std::string & CMN_UNUSED(filename))
{
}

void displayTask::Startup(void)
{
    int numJoints = 0;
    Robot.GetNumberOfJoints(numJoints);
    posRaw.SetSize(numJoints);
    velRaw.SetSize(numJoints);
    analogInRaw.SetSize(numJoints);
    motorFeedbackCurrentRaw.SetSize(numJoints);
    motorControlCurrentRaw.SetSize(numJoints);

    initscr();
    cbreak();
    keypad(stdscr, TRUE);
    noecho();
    nodelay(stdscr, TRUE);
    mvprintw(1, 9, "Robot Sensor Display");
}

void displayTask::Run(void)
{
    size_t i;
    const int ESC_CHAR = 0x1b;
    int c = getch();
    if (c == ESC_CHAR)
        Kill();
    else if (c == 'p') {
        //power_on = !power_on;
    }
    else if (c == '+') {
        for (i = 0; i < motorControlCurrentRaw.size(); i++)
            motorControlCurrentRaw[i] += 0x100;   // 0x100 is about 50 mA
    }
    else if (c == '-') {
        for (i = 0; i < motorControlCurrentRaw.size(); i++)
            motorControlCurrentRaw[i] -= 0x100;   // 0x100 is about 50 mA
    }

    if (!debugStream.str().empty()) {
        int cur_line = DEBUG_START_LINE;
        char line[80];
        memset(line, ' ', sizeof(line));
        for (i = cur_line; i < last_debug_line; i++)
            mvprintw(i, 9, line);
        while (!debugStream.eof()) {
            debugStream.getline(line, sizeof(line));
            mvprintw(cur_line++, 9, line);
        }
        debugStream.clear();
        debugStream.str("");
        last_debug_line = cur_line;
    }

    bool flag;
    Robot.IsValid(flag);
    if (flag) {
        Robot.GetPositionRaw(posRaw);
        Robot.GetVelocityRaw(velRaw);
        Robot.GetAnalogInRaw(analogInRaw);
        Robot.GetMotorCurrentRaw(motorFeedbackCurrentRaw);
        for (i = 0; i < posRaw.size(); i++) {
            mvprintw( 5, 9+5+i*13, "0x%07X", posRaw[i]);
            mvprintw( 6, 9+8+i*13, "0x%04X", velRaw[i]);
            mvprintw( 7, 9+8+i*13, "0x%04X", analogInRaw[i]);
            mvprintw( 8, 9+8+i*13, "0x%04X", motorFeedbackCurrentRaw[i]);
            mvprintw( 9, 9+8+i*13, "0x%04X", motorControlCurrentRaw[i]);
        }
        Robot.SetMotorCurrentRaw(motorControlCurrentRaw);
    }

    refresh();
    osaSleep(0.1);
}

void displayTask::Cleanup(void)
{
    endwin();
}
