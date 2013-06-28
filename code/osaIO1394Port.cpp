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

#include <sawRobotIO1394/osaIO1394Port.h>
#include <stdexcept>

using namespace sawRobotIO1394;
using namespace osaIO1394;

osaIO1394Port::osaIO1394Port(int portNumber, std::ostream & messageStream)
{
    // Construct handle to firewire port
    Port_ = new FirewirePort(portNumber, messageStream);

    // Check number of port users
    if (Port_->NumberOfUsers() > 1) {
        std::ostringstream oss;
        oss << "osaIO1394Port: Found more than one user on firewire port: " << portNumber;
        throw std::runtime_error(oss.str());
    }
}

void osaIO1394Port::Configure(const osaIO1394::Configuration & config)
{
    // Add all the robots
    for (std::vector<osaIO1394::RobotConfiguration>::const_iterator it = config.Robots.begin();
         it != config.Robots.end();
         ++it) {
        this->AddRobot(*it);
    }

    // Add all the digital inputs
    for (std::vector<osaIO1394::DigitalInputConfiguration>::const_iterator it = config.DigitalInputs.begin();
         it != config.DigitalInputs.end();
         ++it) {
        this->AddDigitalInput(*it);
    }
}

void osaIO1394Port::AddRobot(osaIO1394Robot * robot)
{
    if (robot == 0) {
        throw osaIO1394::configuration_error("Robot pointer is null.");
    }

    const osaIO1394::RobotConfiguration & config = robot->GetConfiguration();

    // Check to make sure this robot isn't alreay added
    if (RobotsByName_.count(config.Name) > 0) {
        throw osaIO1394::configuration_error("Robot name is not unique.");
    }

    // Construct a vector of boards relevant to this robot
    std::vector<AmpIO*> robot_boards(config.NumberOfActuators);

    for (int i=0; i < config.NumberOfActuators; i++) {
        int board_id = config.Actuators[i].BoardID;

        // If the board hasn't been created, construct it and add it to the port
        if (Boards_.count(board_id) == 0) {
            Boards_[board_id] = new AmpIO(board_id);
            Port_->AddBoard(Boards_[board_id]);
        }

        // Add the board to the list of boards relevant to this robot
        robot_boards[i] = Boards_[board_id];
    }

    // Set the robot boards
    robot->SetBoards(robot_boards);

    // Store the robot by name
    Robots_.push_back(robot);
    RobotsByName_[config.Name] = robot;
}

void osaIO1394Port::AddRobot(const osaIO1394::RobotConfiguration & config)
{
    osaIO1394Robot * robot = new osaIO1394Robot(config);
    this->AddRobot(robot);
}

osaIO1394Robot * osaIO1394Port::Robot(const std::string & name)
{
    return RobotsByName_.at(name);
}

const osaIO1394Robot * osaIO1394Port::Robot(const std::string & name) const
{
    return RobotsByName_.at(name);
}

osaIO1394Robot * osaIO1394Port::Robot(const int i)
{
    return Robots_[i];
}

const osaIO1394Robot * osaIO1394Port::Robot(const int i) const
{
    return Robots_[i];
}

void osaIO1394Port::AddDigitalInput(const osaIO1394::DigitalInputConfiguration & config)
{
    osaIO1394DigitalInput * digitalInput = new osaIO1394DigitalInput(config);
    this->AddDigitalInput(digitalInput);
}

void osaIO1394Port::AddDigitalInput(osaIO1394DigitalInput * digitalInput)
{
    if (digitalInput == 0) {
        throw osaIO1394::configuration_error("Digital input pointer is null.");
    }

    const osaIO1394::DigitalInputConfiguration & config = digitalInput->Configuration();

    // Check to make sure this digital_input isn't alreay added
    if (DigitalInputsByName_.count(config.Name) > 0) {
        throw osaIO1394::configuration_error("Digital input name is not unique.");
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

osaIO1394Port::~osaIO1394Port() 
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

void osaIO1394Port::Read(void)
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
    }

    // Poll the state for each digital input
    for (digital_input_iterator digitalInput = DigitalInputs_.begin();
         digitalInput != DigitalInputs_.end();
         ++digitalInput) {
        // Poll this robot's state
        (*digitalInput)->PollState();
    }
}

void osaIO1394Port::Write(void)
{
    // Write to all boards
    Port_->WriteAllBoards();
}

int osaIO1394Port::NumberOfBoards(void) const {
    return Boards_.size();
}

int osaIO1394Port::NumberOfRobots(void) const {
    return Robots_.size();
}

int osaIO1394Port::NumberOfDigitalInputs(void) const {
    return DigitalInputs_.size();
}

void osaIO1394Port::GetRobotNames(std::vector<std::string> & names) const
{
    names.clear();
    for (robot_const_iterator robot = Robots_.begin();
         robot != Robots_.end();
         ++robot) {
        names.push_back((*robot)->Name());
    }
}

void osaIO1394Port::GetDigitalInputNames(std::vector<std::string> & names) const
{
    names.clear();
    for (digital_input_const_iterator digitalInput = DigitalInputs_.begin();
         digitalInput != DigitalInputs_.end();
         ++digitalInput) {
        names.push_back((*digitalInput)->Name());
    }
}
