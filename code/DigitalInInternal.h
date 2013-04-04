/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: RobotInternal.h 4031 2013-03-26 15:14:21Z adeguet1 $

  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _DigitalInInternal_h_
#define _DigitalInInternal_h_

#include <string>
#include <vector>

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstCommon/cmnXMLPath.h>

#include <cisstParameterTypes/prmEventButton.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <AmpIO.h>

#include <stdint.h> // for uint32_t

class mtsInterfaceProvided;
class mtsStateTable;
class AmpIO;

// This DigitalInInternal class is for single digital input.
// It will create a provided interface and eventProvider for this input.
//I guess we need to have a mask for bit position.

class mtsRobotIO1394::DigitalInInternal {
protected:

    /*! Pointer on existing services.  This allows to use the class
      name and level of detail of another class, e.g. the class that
      owns this map.  To set the "Owner", use the method SetOwner
      after the cmnNamedMap is constructed. */
    const cmnClassServicesBase * OwnerServices;

    /*! Method use to emulate the cmnGenericObject interface used by
      CMN_LOG_CLASS macros. */
    //@{
    inline const cmnClassServicesBase * Services(void) const {
        return this->OwnerServices;
    }

    inline cmnLogger::StreamBufType * GetLogMultiplexer(void) const {
        return cmnLogger::GetMultiplexer();
    }
    //@}

    std::string inputName;      // Input Name (from config file)
    AmpIO *board;               // Board Assignment
    int bitID;                  // Board assigned bitID for this Digital Input
    bool pressDir;              // Boolean Flag for Active High(true)/Active Low(false)
    bool triggerPress;          // Boolean Flag for Press Trigger Setting
    bool triggerRelease;        // Boolean Flag for Release Trigger Setting
    AmpIO_UInt32 mask;          // BitMask for this input. From DigitalInput Stream.
    mtsFunctionWrite Button;    // The event funciton for button, will return prmEventButton

    // State data
    bool curValue;              // Current read value
    bool previousValue;         // Saved value from the previous read

    // Methods for provided interface
    void SetupMasks(int bitNumber, AmpIO_UInt32 &maskResult);
public:
    DigitalInInternal(const cmnGenericObject & owner, const std::string &name, AmpIO *board, const int bitID);
    ~DigitalInInternal();
    void Configure(cmnXMLPath &xmlConfigFile, int inputNumber);
    void SetupStateTable(mtsStateTable &stateTable);
    void SetupProvidedInterface(mtsInterfaceProvided *prov, mtsStateTable &stateTable);
    void GetData(void);

};

#endif // __DigitalInInternal_h_
