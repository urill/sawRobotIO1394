/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsRobotIO1394Test.h 3034 2011-10-09 01:53:36Z adeguet1 $

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


#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>

class mtsRobotIO1394Test : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(mtsRobotIO1394Test);
    {
        CPPUNIT_TEST(TestCreate);
    }
    CPPUNIT_TEST_SUITE_END();

 public:
    void setUp(void) {
    }

    void tearDown(void) {
    }

    /*! Test constructor */
    void TestCreate(void);
};


CPPUNIT_TEST_SUITE_REGISTRATION(mtsRobotIO1394Test);

