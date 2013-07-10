/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "FirewirePort.h"
#include "AmpIO.h"

#include <sawRobotIO1394/osaPort1394.h>
#include <stdexcept>
#include <exception>

using namespace sawRobotIO1394;

osaPort1394::osaPort1394(int portNumber, std::ostream & messageStream)
{
    // Construct handle to firewire port
    Port_ = new FirewirePort(portNumber, messageStream);

    // Check number of port users
    if (Port_->NumberOfUsers() > 1) {
        std::ostringstream oss;
        oss << "osaIO1394Port: Found more than one user on firewire port: " << portNumber;
        cmnThrow(osaRuntimeError1394(oss.str()));
    }
}

void osaPort1394::Configure(const osaPort1394Configuration & config)
{
    // Add all the robots
    for (std::vector<osaRobot1394Configuration>::const_iterator it = config.Robots.begin();
         it != config.Robots.end();
         ++it) {
        osaRobot1394 * robot = new osaRobot1394(*it);
        this->AddRobot(robot);
    }

    // Add all the digital inputs
    for (std::vector<osaDigitalInput1394Configuration>::const_iterator it = config.DigitalInputs.begin();
         it != config.DigitalInputs.end();
         ++it) {
        osaDigitalInput1394 * digitalInput = new osaDigitalInput1394(*it);
        this->AddDigitalInput(digitalInput);
    }
}

void osaPort1394::AddRobot(osaRobot1394 * robot)
{
    if (robot == 0) {
        cmnThrow(osaRuntimeError1394("Robot pointer is null."));
    }

    const osaRobot1394Configuration & config = robot->GetConfiguration();

    // Check to make sure this robot isn't already added
    if (RobotsByName_.count(config.Name) > 0) {
        cmnThrow(osaRuntimeError1394("Robot name is not unique."));
    }

    // Construct a vector of boards relevant to this robot
    std::vector<osaActuatorMapping> robot_boards(config.NumberOfActuators);

    for (int i=0; i < config.NumberOfActuators; i++) {
        int board_id = config.Actuators[i].BoardID;

        // If the board hasn't been created, construct it and add it to the port
        if (Boards_.count(board_id) == 0) {
            Boards_[board_id] = new AmpIO(board_id);
            Port_->AddBoard(Boards_[board_id]);
        }

        // Add the board to the list of boards relevant to this robot
        robot_boards[i].board = Boards_[board_id];
        robot_boards[i].axis = config.Actuators[i].AxisID;
    }

    // Set the robot boards
    robot->SetBoards(robot_boards);

    // Store the robot by name
    Robots_.push_back(robot);
    RobotsByName_[config.Name] = robot;
}

osaRobot1394 * osaPort1394::Robot(const std::string & name)
{
    return RobotsByName_.at(name);
}

const osaRobot1394 * osaPort1394::Robot(const std::string & name) const
{
    return RobotsByName_.at(name);
}

osaRobot1394 * osaPort1394::Robot(const int i)
{
    return Robots_[i];
}

const osaRobot1394 * osaPort1394::Robot(const int i) const
{
    return Robots_[i];
}

void osaPort1394::AddDigitalInput(osaDigitalInput1394 * digitalInput)
{
    if (digitalInput == 0) {
        cmnThrow(osaRuntimeError1394("Digital input pointer is null."));
    }

    const osaDigitalInput1394Configuration & config = digitalInput->Configuration();

    // Check to make sure this digital_input isn't already added
    if (DigitalInputsByName_.count(config.Name) > 0) {
        cmnThrow(osaRuntimeError1394("Digital input name is not unique."));
    }

    // Construct a vector of boards relevant to this digital_input
    int boardID = config.BoardID;

    // If the board hasn't been created, construct it and add it to the port
    if (Boards_.count(boardID) == 0) {
        Boards_[boardID] = new AmpIO(boardID);
        Port_->AddBoard(Boards_[boardID]);
    }

    // Assign the board to the digital input
    digitalInput->SetBoard(Boards_[boardID]);

    // Store the digital_input by name
    DigitalInputs_.push_back(digitalInput);
    DigitalInputsByName_[config.Name] = digitalInput;
}

osaPort1394::~osaPort1394()
{
    // Delete robots before deleting boards
    for (robot_iterator robot = Robots_.begin();
         robot != Robots_.end();
         ++robot) {
        if (*robot != 0) {
            delete *robot;
        }
    }
    Robots_.clear();

    // Delete digital inputs before deleting boards
    for (digital_input_iterator digitalInput = DigitalInputs_.begin();
         digitalInput != DigitalInputs_.end();
         ++digitalInput) {
        if (*digitalInput != 0) {
            delete *digitalInput;
        }
    }
    DigitalInputs_.clear();

    // Delete board structures
    for (board_iterator board = Boards_.begin();
         board != Boards_.end();
         ++board) {
        if (board->second != 0) {
            Port_->RemoveBoard(board->first);
            delete board->second;
        }
    }
    Boards_.clear();

    // Delete firewire port
    if (Port_ != 0) {
        delete Port_;
    }
}

void osaPort1394::Read(void)
{
    // Read from all boards on the port
    Port_->ReadAllBoards();

    // Poll the state for each robot
    for (robot_iterator robot = Robots_.begin();
         robot != Robots_.end();
         ++robot) {
        // Poll the board validity
        (*robot)->PollValidity();

        // Poll this robot's state
        (*robot)->PollState();

        // Convert bits to usable numbers
        (*robot)->ConvertState();

        // Perform post conversion checks and computations
        try{
            (*robot)->CheckState();
        }catch (std::exception& e){
            std::cerr << e.what() << std::endl;
        }
    }

    // Poll the state for each digital input
    for (digital_input_iterator digitalInput = DigitalInputs_.begin();
         digitalInput != DigitalInputs_.end();
         ++digitalInput) {
        // Poll this robot's state
        (*digitalInput)->PollState();
    }
}

void osaPort1394::Write(void)
{
    // Write to all boards
    Port_->WriteAllBoards();
}

int osaPort1394::NumberOfBoards(void) const {
    return Boards_.size();
}

int osaPort1394::NumberOfRobots(void) const {
    return Robots_.size();
}

int osaPort1394::NumberOfDigitalInputs(void) const {
    return DigitalInputs_.size();
}

void osaPort1394::GetRobotNames(std::vector<std::string> & names) const
{
    names.clear();
    for (robot_const_iterator robot = Robots_.begin();
         robot != Robots_.end();
         ++robot) {
        names.push_back((*robot)->Name());
    }
}

void osaPort1394::GetDigitalInputNames(std::vector<std::string> & names) const
{
    names.clear();
    for (digital_input_const_iterator digitalInput = DigitalInputs_.begin();
         digitalInput != DigitalInputs_.end();
         ++digitalInput) {
        names.push_back((*digitalInput)->Name());
    }
}
