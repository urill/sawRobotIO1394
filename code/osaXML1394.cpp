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


#include <sawRobotIO1394/osaXML1394.h>

namespace sawRobotIO1394 {

    void osaXML1394ConfigurePort(const std::string & filename, osaPort1394Configuration & config)
    {
        cmnXMLPath xmlConfig;
        xmlConfig.SetInputSource(filename);

        // Get the number of robot elements
        int num_robots = 0;
        xmlConfig.GetXMLValue("", "count(/Config/Robot)",num_robots);

        for (int i = 0; i < num_robots; i++) {
            osaRobot1394Configuration robot;

            // Store the robot in the config if it's succesfully parsed
            if (osaXML1394ConfigureRobot(xmlConfig, i + 1, robot)) {
                config.Robots.push_back(robot);
            }
        }

        // Get the number of digital input elements
        int num_digital_inputs = 0;
        xmlConfig.GetXMLValue("", "count(/Config/DigitalIn)", num_digital_inputs);

        for (int i = 0; i < num_digital_inputs; i++) {
            osaDigitalInput1394Configuration digital_input;

            // Store the digital_input in the config if it's succesfully parsed
            if (osaXML1394ConfigureDigitalInput(xmlConfig, i + 1, digital_input)) {
                config.DigitalInputs.push_back(digital_input);
            }
        }
    }

    bool osaXML1394ConfigureRobot(cmnXMLPath & xmlConfig,
                                  const int robotIndex,
                                  osaRobot1394Configuration & robot)
    {
        bool tagsFound = true;
        char path[64];
        const char * context = "Config";

        sprintf(path, "Robot[%d]/@Name", robotIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, robot.Name);

        sprintf(path, "Robot[%d]/@NumOfActuator", robotIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, robot.NumberOfActuators);

        sprintf(path, "Robot[%d]/@NumOfJoint", robotIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, robot.NumberOfJoints);

        for (int i = 0; i < robot.NumberOfActuators; i++) {
            osaActuator1394Configuration actuator;
            int actuator_index = i + 1;

            sprintf(path,"Robot[%d]/Actuator[%d]/@BoardID", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.BoardID);
            if ((actuator.BoardID < 0) || (actuator.BoardID >= (int)MAX_BOARDS)) {
                CMN_LOG_RUN_ERROR << "Configure: invalid board number " << actuator.BoardID
                                  << " for board " << i << std::endl;
                return false;
            }

            sprintf(path,"Robot[%d]/Actuator[%d]/@AxisID", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.AxisID);
            if ((actuator.AxisID < 0) || (actuator.AxisID >= (int)MAX_AXES)) {
                CMN_LOG_RUN_ERROR << "Configure: invalid axis number " << actuator.AxisID
                                  << " for actuator " << i << std::endl;
                return false;
            }

            std::string actuator_type = "";
            sprintf(path,"Robot[%d]/Actuator[%d]/@Type", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator_type);
            if (actuator_type == "") {
                CMN_LOG_RUN_WARNING << "Configure: no actuator type specified " << actuator.AxisID
                                    << " for actuator " << i << " set to Revolute by default" << std::endl;
                actuator_type = "Revolute";
            }
            double unitPosConversion;
            if (actuator_type == "Revolute") {
                // deg to radian
                unitPosConversion = cmnPI_180;
                actuator.JointType = PRM_REVOLUTE;
            } else if (actuator_type == "Prismatic") {
                unitPosConversion = cmn_mm;
                actuator.JointType = PRM_PRISMATIC;
            }

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/AmpsToBits/@Scale", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Drive.CurrentToBitsScale);

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/AmpsToBits/@Offset", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Drive.CurrentToBitsOffset);

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/BitsToFeedbackAmps/@Scale", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Drive.BitsToCurrentScale);

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/BitsToFeedbackAmps/@Offset", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Drive.BitsToCurrentOffset);

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/NmToAmps/@Scale", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Drive.EffortToCurrentScale);

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/MaxCurrent/@Value", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Drive.ActuatorCurrentCommandLimit);

            sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToPosSI/@Scale", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Encoder.BitsToPositionScale);
            actuator.Encoder.BitsToPositionScale *= unitPosConversion; // -------------------------------------------- adeguet1, make sure these are degrees

            sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToPosSI/@Offset", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Encoder.BitsToPositionOffset);
            actuator.Encoder.BitsToPositionOffset *= unitPosConversion; // -------------------------------------------- adeguet1, make sure these are degrees

            sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaPosSI/@Scale", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Encoder.BitsToDPositionScale);

            sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaPosSI/@Offset", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Encoder.BitsToDPositionOffset);

            sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaT/@Scale", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Encoder.BitsToDTimeScale);

            sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToDeltaT/@Offset", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Encoder.BitsToDTimeOffset);

            sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/CountsPerTurn/@Value", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Encoder.CountsPerTurn);

            sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/BitsToVolts/@Scale", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Pot.BitsToVoltageScale);

            sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/BitsToVolts/@Offset", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Pot.BitsToVoltageOffset);

            sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/VoltsToPosSI/@Scale", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Pot.VoltageToPositionScale);
            actuator.Pot.VoltageToPositionScale *= unitPosConversion; // -------------------------------------------- adeguet1, make sure these are degrees

            sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/VoltsToPosSI/@Offset", robotIndex, actuator_index);
            xmlConfig.GetXMLValue(context, path, actuator.Pot. VoltageToPositionOffset);
            actuator.Pot.VoltageToPositionOffset *= unitPosConversion; // -------------------------------------------- adeguet1, make sure these are degrees

            // Add the actuator
            robot.Actuators.push_back(actuator);
        }

        // look for potentiometers position, if any
        std::string potentiometerPosition;
        sprintf(path,"Robot[%d]/Potentiometers/@Position", robotIndex);
        if (xmlConfig.GetXMLValue(context, path, potentiometerPosition)) {
            if (potentiometerPosition == "Actuators") {
                robot.PotLocation = POTENTIOMETER_ON_ACTUATORS;
            } else if (potentiometerPosition == "Joints") {
                robot.PotLocation = POTENTIOMETER_ON_JOINTS;
            } else {
                CMN_LOG_RUN_ERROR << "Configure: invalid <Potentiometers Position=\"\"> value, must be either \"Joints\" or \"Actuators\" for robot number "
                                  << robotIndex << std::endl;
            }
        } else {
            CMN_LOG_RUN_VERBOSE << "Configure: no <Potentiometers Position=\"\"> found." << std::endl;
        }

        // Configure Coupling
        if (!osaXML1394ConfigureCoupling(xmlConfig, robotIndex, robot)) {
            return false;
        }

        if (!tagsFound) {
            return false;
        }

        return true;
    }

    bool osaXML1394ConfigureCoupling(cmnXMLPath & xmlConfig,
                                     const int robotIndex,
                                     osaRobot1394Configuration & robot)
    {
        char path[64];
        const char * context = "Config";
        //The Coupling Value must be equal to 1 for this configuration to work.
        int coupling = 0;
        sprintf(path, "Robot[%i]/Coupling/@Value", robotIndex);
        xmlConfig.GetXMLValue(context, path, coupling);
        robot.HasActuatorToJointCoupling = (coupling == 1);

        if (robot.HasActuatorToJointCoupling) {
            bool parse_success = true;

            robot.ActuatorToJointPosition.SetSize(robot.NumberOfJoints, robot.NumberOfActuators, 0.0);
            robot.JointToActuatorPosition.SetSize(robot.NumberOfActuators, robot.NumberOfJoints, 0.0);
            robot.ActuatorToJointEffort.SetSize(robot.NumberOfJoints, robot.NumberOfActuators, 0.0);
            robot.JointToActuatorEffort.SetSize(robot.NumberOfActuators, robot.NumberOfJoints, 0.0);

            parse_success &= osaXML1394ConfigureCouplingMatrix(xmlConfig, robotIndex, "ActuatorToJointPosition",
                                                               robot.NumberOfJoints, robot.NumberOfActuators,
                                                               robot.ActuatorToJointPosition);

            parse_success &= osaXML1394ConfigureCouplingMatrix(xmlConfig, robotIndex, "JointToActuatorPosition",
                                                               robot.NumberOfActuators, robot.NumberOfJoints,
                                                               robot.JointToActuatorPosition);

            parse_success &= osaXML1394ConfigureCouplingMatrix(xmlConfig, robotIndex, "ActuatorToJointTorque",
                                                               robot.NumberOfJoints, robot.NumberOfActuators,
                                                               robot.ActuatorToJointEffort);

            parse_success &= osaXML1394ConfigureCouplingMatrix(xmlConfig, robotIndex, "JointToActuatorTorque",
                                                               robot.NumberOfActuators, robot.NumberOfJoints,
                                                               robot.JointToActuatorEffort);
            // Still need to do proper alignment and such for joint/actuator situ for each matrix.
            if (!parse_success) {
                return false;
            }
        }

        // make sure the coupling matrices make sense
        vctDoubleMat product, identity;

        identity.ForceAssign(vctDoubleMat::Eye(robot.NumberOfActuators));

        product.SetSize(robot.NumberOfActuators, robot.NumberOfActuators);
        product.ProductOf(robot.ActuatorToJointPosition, robot.JointToActuatorPosition);

        if (!product.AlmostEqual(identity, 0.001)) {
            CMN_LOG_RUN_ERROR << "ConfigureCoupling: product of position coupling matrices not identity:"
                              << std::endl << product << std::endl;
            return false;
        }

        product.ProductOf(robot.ActuatorToJointEffort, robot.JointToActuatorEffort);
        if (!product.AlmostEqual(identity, 0.001)) {
            CMN_LOG_RUN_ERROR << "ConfigureCoupling: product of torque coupling matrices not identity:"
                              << std::endl << product << std::endl;
            return false;
        }

        return true;
    }

    bool osaXML1394ConfigureCouplingMatrix(cmnXMLPath & xmlConfig,
                                           const int robotIndex,
                                           const char * couplingString,
                                           int numRows,
                                           int numCols,
                                           vctDoubleMat & resultMatrix)
    {
        std::string context = "Config";

        // Construct XPath for matrix
        std::ostringstream matrixPath;
        matrixPath << "Robot[" << robotIndex << "]/Coupling/" << couplingString;

        for (int i = 0; i < numRows; i++) {
            // Construct XPath for matrix row
            std::ostringstream rowPath;
            rowPath << matrixPath.str() << "/Row[" << i + 1 << "]/@Val";

            // Get the matrix row text
            std::string rowAsString = "";
            xmlConfig.GetXMLValue(context, rowPath.str(), rowAsString);

            // Convert the text to a cisstVector row
            std::stringstream rowAsStringStream;
            rowAsStringStream.str(rowAsString);

            vctDoubleVec row;
            row.SetSize(numCols);

            if (!row.FromStreamRaw(rowAsStringStream)) {
                CMN_LOG_RUN_ERROR << "Row vector Assign failed on row " << i << ", path: " << rowPath.str() << std::endl;
                return false;
            }
            resultMatrix.Row(i).Assign(row);
        }
        return true;
    }

    bool osaXML1394ConfigureDigitalInput(cmnXMLPath & xmlConfig,
                                         const int tagIndex,
                                         osaDigitalInput1394Configuration & digital_input)
    {
        // Digital Input Setup Stage
        char path[64];
        const char * context = "Config";
        bool tagsFound = true;

        //Check there is digital input entry. Return boolean result for success/fail.
        sprintf(path,"DigitalIn[%i]/@Name", tagIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, digital_input.Name);
        sprintf(path,"DigitalIn[%i]/@BoardID", tagIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, digital_input.BoardID);
        sprintf(path,"DigitalIn[%i]/@BitID", tagIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, digital_input.BitID);

        if (!tagsFound) {
            CMN_LOG_RUN_ERROR << "Configuration for " << path << " failed. Stopping config." << std::endl;
            return false;
        }

        int pressed_value = 0;
        sprintf(path, "DigitalIn[%i]/@Pressed", tagIndex);
        xmlConfig.GetXMLValue(context,path, pressed_value);
        digital_input.PressedValue = bool(pressed_value);

        std::string trigger_modes;
        sprintf(path, "DigitalIn[%i]/@Trigger", tagIndex);
        xmlConfig.GetXMLValue(context, path, trigger_modes);

        digital_input.TriggerWhenPressed = false;
        digital_input.TriggerWhenReleased = false;

        if (trigger_modes == "all") {
            digital_input.TriggerWhenPressed = true;
            digital_input.TriggerWhenReleased = true;
        }
        else if (trigger_modes == "press") {
            digital_input.TriggerWhenPressed = true;
        }
        else if (trigger_modes == "release") {
            digital_input.TriggerWhenReleased = true;
        }
        else if (trigger_modes != "none") {
            //Should not come here during init.
            CMN_LOG_RUN_ERROR << "Unacceptable Trigger argument: " <<trigger_modes << "." <<std::endl;
            CMN_LOG_RUN_ERROR << "Trigger argument should be one of these: [all,press,release,none]." << std::endl;
            return false;
        }

        return true;
    }

}
