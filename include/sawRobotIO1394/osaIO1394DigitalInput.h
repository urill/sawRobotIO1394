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

#ifndef _osaIO1394DigitalInput_h
#define _osaIO1394DigitalInput_h

#include <sawRobotIO1394/osaRobotIO1394.h>
#include "AmpIO.h"

namespace sawRobotIO1394 {
    class osaIO1394DigitalInput {
    public:
        osaIO1394DigitalInput(const osaIO1394::DigitalInputConfiguration & config);

        void Configure(const osaIO1394::DigitalInputConfiguration & config);
        void SetBoard(AmpIO * board);

        void PollState(void);

        osaIO1394::DigitalInputConfiguration Configuration(void) const;

        std::string Name(void) const;
        bool Value(void) const;
        bool PreviousValue(void) const;

    protected:
        AmpIO * Board_;              // Board Assignment

        osaIO1394::DigitalInputConfiguration Configuration_;
        std::string Name_;
        int BitID_;                  // Board assigned bitID for this Digital Input
        AmpIO_UInt32 BitMask_;       // BitMask for this input. From DigitalInput Stream.
        bool PressedValue_;          // Boolean Flag for Active High(true)/Active Low(false)
        bool TriggerPress_;          // Boolean Flag for Press Trigger Setting
        bool TriggerRelease_;        // Boolean Flag for Release Trigger Setting

        // State data
        AmpIO_UInt32 DigitalInputBits_; // BitMask for this input. From DigitalInput Stream.
        bool Value_;                    // Current read value
        bool PreviousValue_;            // Saved value from the previous read
    };
}

#endif // _osaIO1394DigitalInput_h
