/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsRobotIO1394Test.cpp 3520 2012-03-06 16:10:40Z adeguet1 $

  Author(s):  Anton Deguet
  Created on: 2013-03-01

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include "mtsRobotIO1394Test.h"


void mtsRobotIO1394Test::TestCreate(void) {
    mtsRobotIO1394 * robot = new mtsRobotIO1394("robot", 1.0 * cmn_ms, 0);
    CPPUNIT_ASSERT(robot);
}
