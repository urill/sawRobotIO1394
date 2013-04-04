/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

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
    debugStream(std::stringstream::out|std::stringstream::in),
    last_debug_line(DEBUG_START_LINE), power_on(false), safety_relay(false), watchdog(false),
    payloadCount(0), pressCount(0), releaseCount(0)
{
    mtsInterfaceRequired *req = AddInterfaceRequired("Robot");
    if (req) {
        req->AddFunction("GetNumberOfActuators", Robot.GetNumberOfActuators);
        req->AddFunction("IsValid", Robot.IsValid);
        req->AddFunction("EnablePower", Robot.EnablePower);
        req->AddFunction("DisablePower", Robot.DisablePower);
        req->AddFunction("EnableSafetyRelay", Robot.EnableSafetyRelay);
        req->AddFunction("DisableSafetyRelay", Robot.DisableSafetyRelay);
        req->AddFunction("GetPositionEncoderRaw", Robot.GetPositionEncoderRaw);
        req->AddFunction("GetVelocityRaw", Robot.GetVelocityRaw);
        req->AddFunction("GetAnalogInputRaw", Robot.GetAnalogInputRaw);
        req->AddFunction("GetMotorFeedbackCurrentRaw", Robot.GetMotorCurrentRaw);
        req->AddFunction("GetAmpEnable", Robot.GetAmpEnable);
        req->AddFunction("GetAmpStatus", Robot.GetAmpStatus);
        req->AddFunction("GetPowerStatus", Robot.GetPowerStatus);
        req->AddFunction("GetSafetyRelay", Robot.GetSafetyRelay);
        req->AddFunction("SetMotorCurrentRaw", Robot.SetMotorCurrentRaw);
        req->AddFunction("SetWatchdogPeriod", Robot.SetWatchdogPeriod);
    }
}

displayTask::~displayTask()
{
}

void displayTask::payloadButtonHandler(const prmEventButton &payload) {
    // Hey, there is an event with button payload coming in! Let's see what it is!
    // Uh... is there a way I can tell where the event came from?
    switch(payload.Type())
    {
    case prmEventButton::PRESSED :
        pressCount = pressCount + 1;
        break;
    case prmEventButton::RELEASED:
        releaseCount = releaseCount + 1;
        break;
    default:
        //Do nothing. Throw error since no other type is expected.
        CMN_LOG_RUN_WARNING<<"Unacceptable eventButton type received. The type is "<<payload<<std::endl;
    }
    payloadCount = payloadCount + 1;
}

void displayTask::Configure(const std::string & CMN_UNUSED(filename))
{
    // Hard-coded to have 12 digital inputs available.
    Configure(12);
}

void displayTask::Configure(const int numberOfDigitalInputs) {
    // Set required interface vector and list of
    digitalInReq.resize(numberOfDigitalInputs);
    DigitalInInterfaceList.resize(numberOfDigitalInputs);

    //Iteratively generate required interface for digitalInput.c
    int k = 0;
    for (k = 0; k < numberOfDigitalInputs; k++) {
        std::string DInputName = "DigitalIn-";
        std::stringstream ss;
        ss<<k;
        DInputName.append(ss.str());
        digitalInReq[k] = AddInterfaceRequired(DInputName);
        if(digitalInReq[k]){
            digitalInReq[k]->AddFunction("GetButton",DigitalInInterfaceList[k].GetButton);
            digitalInReq[k]->AddEventHandlerWrite(&displayTask::payloadButtonHandler,this,"Button");
        }
    }
}

void displayTask::Startup(void)
{
    int numActuators = 0;
    Robot.GetNumberOfActuators(numActuators);
    posRaw.SetSize(numActuators);
    velRaw.SetSize(numActuators);
    analogInRaw.SetSize(numActuators);
    motorFeedbackCurrentRaw.SetSize(numActuators);
    ampEnable.SetSize(numActuators);
    ampStatus.SetSize(numActuators);
    motorControlCurrentRaw.SetSize(numActuators);
    motorControlCurrentRaw.SetAll(0x8000);

    initscr();
    cbreak();
    keypad(stdscr, TRUE);
    noecho();
    nodelay(stdscr, TRUE);
    mvprintw( 1, 4, "Robot Sensor Display (ESC to exit)");
    mvprintw( 2, 4, "Press: p to toggle power, s to toggle safety relay");
    mvprintw( 3, 4, "       +/- to increase/decrease commanded current");
    mvprintw( 5, 4, "Enc Pos");
    mvprintw( 6, 4, "Enc Vel");
    mvprintw( 7, 4, "Analog In");
    mvprintw( 8, 4, "Current FdBk");
    mvprintw( 9, 4, "Current Ctrl");
    mvprintw(10, 4, "Amp Enable");
    mvprintw(11, 4, "Amp Status");
    mvprintw(12, 4, "Power");
    mvprintw(12, 22, "Safety Relay");

    mvprintw(25, 4, "Digital");
    mvprintw(27, 4, "PLCount");
    mvprintw(28, 4, "PressCount");
    mvprintw(29, 4, "ReleaseCount");
}

void displayTask::Run(void)
{
    size_t i;
    const int ESC_CHAR = 0x1b;
    int c = getch();
    if (c == ESC_CHAR)
        Kill();
    else if (c == 'p') {
        if (power_on)
            Robot.DisablePower();
        else
            Robot.EnablePower();
        power_on = !power_on;
    }
    else if (c == 's') {
        if (safety_relay)
            Robot.DisableSafetyRelay();
        else
            Robot.EnableSafetyRelay();
        safety_relay = !safety_relay;
    }
    else if (c == '+') {
        for (i = 0; i < motorControlCurrentRaw.size(); i++)
            motorControlCurrentRaw[i] += 0x100;   // 0x100 is about 50 mA
    }
    else if (c == '-') {
        for (i = 0; i < motorControlCurrentRaw.size(); i++)
            motorControlCurrentRaw[i] -= 0x100;   // 0x100 is about 50 mA
    }
    else if (c == 'w') {
        const unsigned long period_ms = (watchdog ? 0 : 250);
        Robot.SetWatchdogPeriod(period_ms);
        watchdog = !watchdog;
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
        Robot.GetPositionEncoderRaw(posRaw);
        Robot.GetVelocityRaw(velRaw);
        Robot.GetAnalogInputRaw(analogInRaw);
        Robot.GetMotorCurrentRaw(motorFeedbackCurrentRaw);
        Robot.GetAmpEnable(ampEnable);
        Robot.GetAmpStatus(ampStatus);
        Robot.GetPowerStatus(powerStatus);
        Robot.GetSafetyRelay(safetyRelay);
        for (i = 0; i < velRaw.size(); i++) {
            mvprintw( 5, 14+5+i*13, "0x%07X", posRaw[i]);
            mvprintw( 6, 14+8+i*13, "0x%04X", velRaw[i]);
            mvprintw( 7, 14+8+i*13, "0x%04X", analogInRaw[i]);
            mvprintw( 8, 14+8+i*13, "0x%04X", motorFeedbackCurrentRaw[i]);
            mvprintw( 9, 14+8+i*13, "0x%04X", motorControlCurrentRaw[i]);
            mvprintw(10, 14+8+i*13, (ampEnable[i]?" On":"Off"));
            mvprintw(11, 14+8+i*13, (ampStatus[i]?" On":"Off"));
        }
        mvprintw(12, 14, (powerStatus ? " On":"Off"));
        mvprintw(12, 35, ((safetyRelay != 0) ? " On":"Off"));
        Robot.SetMotorCurrentRaw(motorControlCurrentRaw);
    }

    for (i = 0; i < DigitalInInterfaceList.size(); i++) {
        DigitalInInterfaceList[i].GetButton(bDigital);
        mvprintw(25, 14+(2*i),(bDigital?"1":"0"));
    }

    mvprintw(27, 18,"%i", payloadCount);
    mvprintw(28, 18,"%i", pressCount);
    mvprintw(29, 18,"%i", releaseCount);

    this->ProcessQueuedEvents();

    refresh();
    osaSleep(0.1);
}

void displayTask::Cleanup(void)
{
    Robot.DisablePower();
    Robot.DisableSafetyRelay();
    endwin();
}
