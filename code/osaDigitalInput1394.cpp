/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen, Peter Kazanzides, Jonathan Bohren
  Created on: 2011-06-10

  (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawRobotIO1394/osaDigitalInput1394.h>

#include "FirewirePort.h"
#include "AmpIO.h"

using namespace sawRobotIO1394;

osaDigitalInput1394::osaDigitalInput1394(const osaDigitalInput1394Configuration & config):
    DigitalInputBits_(0x0),
    Value_(false),
    PreviousValue_(false)
{
    this->Configure(config);
}

void osaDigitalInput1394::Configure(const osaDigitalInput1394Configuration & config)
{
    // Store configuration
    Configuration_ = config;
    Name_ = config.Name;
    BitID_ = config.BitID;
    BitMask_ = 0x1 << BitID_;
    PressedValue_ = config.PressedValue;
    TriggerPress_ = config.TriggerWhenPressed;
    TriggerRelease_ = config.TriggerWhenReleased;

    // Set the value to un-pressed
    Value_ = !PressedValue_;
    PreviousValue_ = Value_;
}

void osaDigitalInput1394::SetBoard(AmpIO * board)
{
    if (board == 0) {
        cmnThrow(osaRuntimeError1394("Invalid board pointer."));
    }
    Board_ = board;
}

void osaDigitalInput1394::PollState(void)
{
    // Store previous value
    PreviousValue_ = Value_;

    // Get the new value
    DigitalInputBits_ =  Board_->GetDigitalInput();

    // If the masked bit is low, set the value to the pressed value
    Value_ = (DigitalInputBits_ & BitMask_) ? (!PressedValue_) : (PressedValue_);
}

osaDigitalInput1394Configuration osaDigitalInput1394::Configuration(void) const {
    return Configuration_;
}

std::string osaDigitalInput1394::Name(void) const {
    return Name_;
}

bool osaDigitalInput1394::Value(void) const {
    return Value_;
}

bool osaDigitalInput1394::PreviousValue(void) const {
    return PreviousValue_;
}
