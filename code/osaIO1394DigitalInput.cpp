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

#include <sawRobotIO1394/osaIO1394DigitalInput.h>

#include "FirewirePort.h"
#include "AmpIO.h"

using namespace sawRobotIO1394;
using namespace osaIO1394;

osaIO1394DigitalInput::osaIO1394DigitalInput(const osaIO1394::DigitalInputConfiguration & config):
    DigitalInputBits_(0x0),
    Value_(false),
    PreviousValue_(false)
{
    this->Configure(config);
}

void osaIO1394DigitalInput::Configure(const osaIO1394::DigitalInputConfiguration & config)
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

void osaIO1394DigitalInput::SetBoard(AmpIO * board)
{
    if (board == 0) {
        throw osaIO1394::configuration_error("Invalid board pointer.");
    }
    Board_ = board;
}

void osaIO1394DigitalInput::PollState(void)
{
    // Store previous value
    PreviousValue_ = Value_;

    // Get the new value
    DigitalInputBits_ =  Board_->GetDigitalInput();

    // If the masked bit is low, set the value to the pressed value
    Value_ = (DigitalInputBits_ & BitMask_) ? (!PressedValue_) : (PressedValue_);
}

osaIO1394::DigitalInputConfiguration osaIO1394DigitalInput::Configuration(void) const {
    return Configuration_;
}

std::string osaIO1394DigitalInput::Name(void) const {
    return Name_;
}

bool osaIO1394DigitalInput::Value(void) const {
    return Value_;
}

bool osaIO1394DigitalInput::PreviousValue(void) const {
    return PreviousValue_;
}
