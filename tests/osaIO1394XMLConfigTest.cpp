/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Jonathan Bohren
  Created on: 2013-06-29

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnXMLPath.h>

#include <cisstParameterTypes/prmJointType.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

#include <sawRobotIO1394/osaRobotIO1394.h>
#include <sawRobotIO1394/osaIO1394XMLConfig.h>

using namespace sawRobotIO1394;

class osaIO1394XMLConfigTest : public CppUnit::TestFixture
{
protected:
    cmnPath cmn_path;

    CPPUNIT_TEST_SUITE(osaIO1394XMLConfigTest);
    {
        //CPPUNIT_TEST(TestCoupling);
        CPPUNIT_TEST(TestConfigure);
        // CPPUNIT_TEST(TestEncoder);
        // CPPUNIT_TEST(TestDriveAmps);
        // CPPUNIT_TEST(TestDriveNm);
        // CPPUNIT_TEST(TestAnalogIn);
    }
    CPPUNIT_TEST_SUITE_END();

public:

    void setUp(void) {
        cmn_path.AddRelativeToCisstShare("/sawRobotIO1394");
    }

    void tearDown(void) {
    }

    /*! Test constructor */
    //void TestCoupling(void);
    void TestConfigure(void);
    //    void TestEncoder(void);
    //    void TestDriveAmps(void);
    //    void TestDriveNm(void);
    //    void TestAnalogIn(void);
};

CPPUNIT_TEST_SUITE_REGISTRATION(osaIO1394XMLConfigTest);

void osaIO1394XMLConfigTest::TestConfigure(void)
{
    std::string xml_path = cmn_path.Find("sawRobotIO1394TestBoard.xml");

    CPPUNIT_ASSERT(xml_path.length() > 0);

    osaIO1394::Configuration config;
    osaIO1394XMLConfig::ConfigurePort(xml_path, config);

    CPPUNIT_ASSERT(config.Robots.size() == 1);
    CPPUNIT_ASSERT(config.DigitalInputs.size() == 12);

    osaIO1394::RobotConfiguration & robot = config.Robots[0];

    CPPUNIT_ASSERT(robot.Name == "Robot");
    CPPUNIT_ASSERT(robot.Actuators.size() == 4);
    CPPUNIT_ASSERT(robot.NumberOfActuators == 4);
    CPPUNIT_ASSERT(robot.NumberOfJoints == 4);
    CPPUNIT_ASSERT(robot.HasActuatorToJointCoupling == true);
    CPPUNIT_ASSERT(robot.PotLocation == osaIO1394::POTENTIOMETER_ON_ACTUATORS);

    CPPUNIT_ASSERT(robot.ActuatorToJointPosition.rows() == 4);
    CPPUNIT_ASSERT(robot.JointToActuatorPosition.rows() == 4);
    CPPUNIT_ASSERT(robot.ActuatorToJointEffort.rows() == 4);
    CPPUNIT_ASSERT(robot.JointToActuatorEffort.rows() == 4);
    CPPUNIT_ASSERT(robot.ActuatorToJointPosition.cols() == 4);
    CPPUNIT_ASSERT(robot.JointToActuatorPosition.cols() == 4);
    CPPUNIT_ASSERT(robot.ActuatorToJointEffort.cols() == 4);
    CPPUNIT_ASSERT(robot.JointToActuatorEffort.cols() == 4);

    vctDoubleMat actuator_to_joint_position(4, 4, VCT_ROW_MAJOR);
    actuator_to_joint_position.Assign(0.0, 1.0, 0.0, 0.0,
                                      1.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 1.0, 1.0,
                                      0.0, 0.0, 0.0, 1.0);
    CPPUNIT_ASSERT(robot.ActuatorToJointPosition.AlmostEqual(actuator_to_joint_position));
}


#if 0
void osaIO1394XMLConfigTest::TestCoupling(void)
{
    cmnXMLPath xml_config;
    xml_config.SetInputSource(xml_str);

    osaIO1394::RobotInfo robot;
    robot.num_actuators = 4;
    robot.num_joints = 4;

    osaIO1394XMLConfig::ConfigureCoupling(xml_config, 1, robot);
}
#endif
