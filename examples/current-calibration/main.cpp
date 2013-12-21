/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*
  $Id$

  Author(s):  Anton Deguet
  Created on: 2013-12-20

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

// system
#include <iostream>
// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawRobotIO1394/osaConfiguration1394.h>
#include <sawRobotIO1394/osaXML1394.h>
#include <sawRobotIO1394/osaPort1394.h>
#include <sawRobotIO1394/osaRobot1394.h>

using namespace sawRobotIO1394;
const double WatchdogPeriod = 100.0 * cmn_ms;
size_t NumberOfActuators;
vctDoubleVec Zeros;

int main(int argc, char * argv[])
{
    cmnCommandLineOptions options;
    int portNumber = 0;
    std::string configFile;
    options.AddOptionOneValue("c", "config",
                              "configuration file",
                              cmnCommandLineOptions::REQUIRED, &configFile);
    options.AddOptionOneValue("p", "port",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL, &portNumber);
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    if (!cmnPath::Exists(configFile)) {
        std::cerr << "Can't find file \"" << configFile << "\"." << std::endl;
        return -1;
    }
    std::cout << "Configuration file: " << configFile << std::endl
              << "Port: " << portNumber << std::endl;

    std::cout << "Make sure:" << std::endl
              << " - your computer is connected to the firewire controller." << std::endl
              << " - the arm corresponding to the configuration file \"" << configFile << "\" is connected to the controller." << std::endl
              << " - the E-Stop is closed, i.e. will let the controller power on." << std::endl
              << " - you have no other device connected to the firewire chain." << std::endl
              << " - you have no other program trying to communicate with the controller." << std::endl
              << std::endl
              << "Press any key to get started." << std::endl;
    cmnGetChar();

    std::cout << "Loading config file ..." << std::endl;
    osaPort1394Configuration config;
    osaXML1394ConfigurePort(configFile, config);

    std::cerr << "Creating robot ..." << std::endl;
    if (config.Robots.size() == 0) {
        std::cerr << "Error: the config file doesn't define a robot." << std::endl;
        return -1;
    }
    if (config.Robots.size() != 1) {
        std::cerr << "Error: the config file defines more than one robot." << std::endl;
        return -1;
    }
    osaRobot1394 * robot = new osaRobot1394(config.Robots[0]);
    NumberOfActuators = robot->NumberOfActuators();
    Zeros.SetSize(NumberOfActuators);

    std::cerr << "Creating port ..." << std::endl;
    osaPort1394 * port = new osaPort1394(portNumber);
    port->AddRobot(robot);

    std::cout << std::endl
              << "Ready to power?  Press any key to start." << std::endl;
    cmnGetChar();

    std::cout << "Enabling power ..." << std::endl;
    robot->SetWatchdogPeriod(300.0 * cmn_ms);
    robot->SetJointEffort(Zeros);
    robot->EnablePower();
    port->Write();

    // wait a bit to make sure current stabilizes, 500 * 10 ms = 5 seconds
    for (size_t i = 0; i < 500; ++i) {
        robot->SetJointEffort(Zeros);
        osaSleep(10.0 * cmn_ms);
        port->Write();
    }

    // check that power is on
    port->Read();
    if (!robot->PowerStatus()) {
        std::cerr << "Error: unable to power on controllers, make sure E-Stop is ok." << std::endl;
        delete port;
        return -1;
    }
    if (!robot->ActuatorPowerStatus().All()) {
        std::cerr << "Error: failed to turn on actuator power: " << robot->ActuatorPowerStatus() << std::endl;
        delete port;
        return -1;
    }
    std::cout << "Status: power seems fine." << std::endl
              << "Starting calibration ..." << std::endl;
    // collect samples
    const size_t totalSamples = 50000;
    std::vector<vctDoubleVec> samples;
    samples.resize(totalSamples);
    vctDoubleVec sumSamples, averageAllSamples;
    sumSamples.SetSize(NumberOfActuators);
    averageAllSamples.SetSize(NumberOfActuators);
    sumSamples.SetAll(0.0);
    for (size_t index = 0; index < totalSamples; ++index) {
        // write to make sure watchdog is not tripped
        robot->SetJointEffort(Zeros);
        port->Write();
        port->Read();
        samples[index].ForceAssign(robot->ActuatorCurrentFeedback());
        samples[index].Multiply(1000.0); // convert all values to mA to be easier to read
        sumSamples.Add(samples[index]);
    }
    // compute simple average
    averageAllSamples.Assign(sumSamples);
    averageAllSamples.Divide(totalSamples);

    // compute standard deviation
    vctDoubleVec sumDifferencesSquared(NumberOfActuators);
    vctDoubleVec difference(NumberOfActuators);
    for (size_t index = 0; index < totalSamples; ++index) {
        difference.DifferenceOf(samples[index], averageAllSamples);
        sumDifferencesSquared.AddElementwiseProductOf(difference, difference);
    }
    vctDoubleVec stdDeviation(sumDifferencesSquared);
    stdDeviation.Divide(totalSamples);
    for (size_t index = 0; index < stdDeviation.size(); ++index) {
        stdDeviation[index] = sqrt(stdDeviation[index]);
    }

    // eliminate outliers
    vctDoubleVec lower(NumberOfActuators);
    lower.DifferenceOf(averageAllSamples, stdDeviation);
    vctDoubleVec upper(NumberOfActuators);
    upper.SumOf(averageAllSamples, stdDeviation);
    size_t validSamples = 0;
    vctDoubleVec averageValidSamples(NumberOfActuators);
    sumSamples.SetAll(0.0);
    for (size_t index = 0; index < totalSamples; ++index) {
        if (samples[index].ElementwiseLesserOrEqual(upper).All()
            && samples[index].ElementwiseGreaterOrEqual(lower).All()) {
            sumSamples.Add(samples[index]);
            validSamples++;
        }
    }
    averageValidSamples.Assign(sumSamples);
    averageValidSamples.Divide(validSamples);

    // display results
    std::cout << "Status: average current feedback in mA: " << averageAllSamples << std::endl
              << "Status: standard deviation in mA:       " << stdDeviation << std::endl
              << "Status: kept " << validSamples << " samples out of " << totalSamples << std::endl
              << "Status: new average in mA:              " << averageValidSamples << std::endl
              << std::endl
              << "Do you want to update the config file with these values? [Y/y]" << std::endl;

    // save if needed
    char key;
    key = cmnGetChar();
    if ((key == 'y') || (key == 'Y')) {
        cmnXMLPath xmlConfig;
        xmlConfig.SetInputSource(configFile);
        // query previous current offset and scales
        vctDoubleVec previousOffsets(NumberOfActuators, 0.0);
        vctDoubleVec previousScales(NumberOfActuators, 0.0);
        for (size_t index = 0; index < NumberOfActuators; ++index) {
            char path[64];
            const char * context = "Config";
            sprintf(path, "Robot[1]/Actuator[%d]/Drive/AmpsToBits/@Offset", static_cast<int>(index + 1));
            xmlConfig.GetXMLValue(context, path, previousOffsets[index]);
            sprintf(path, "Robot[1]/Actuator[%d]/Drive/AmpsToBits/@Scale", static_cast<int>(index + 1));
            xmlConfig.GetXMLValue(context, path, previousScales[index]);
        }
        // compute new offsets
        vctDoubleVec newOffsets(NumberOfActuators);
        newOffsets.Assign(averageValidSamples);
        newOffsets.Divide(-1000.0); // convert back to Amps and negate
        newOffsets.ElementwiseMultiply(previousScales);
        newOffsets.Add(previousOffsets);

        // ask one last confirmation from user
        std::cout << "Status: current offsets in XML configuration file: " << previousOffsets << std::endl;
        std::cout << "Status: new current offsets:                       " << newOffsets << std::endl
                  << std::endl
                  << "Do you want to save these values? [S/s]" << std::endl;
        key = cmnGetChar();
        if ((key == 's') || (key == 'S')) {
            vctIntVec newOffsetsInt(newOffsets);
            for (size_t index = 0; index < NumberOfActuators; ++index) {
                char path[64];
                const char * context = "Config";
                sprintf(path, "Robot[1]/Actuator[%d]/Drive/AmpsToBits/@Offset", static_cast<int>(index + 1));
                xmlConfig.SetXMLValue(context, path, newOffsetsInt[index]);
            }
            std::string newConfigFile = configFile + "-new";
            xmlConfig.SaveAs(newConfigFile);
            std::cout << "Status: new config file is \"" << newConfigFile << "\"" << std::endl;
        } else {
            std::cout << "Status: user didn't want to save new offsets." << std::endl;
        }
    } else {
        std::cout << "Status: no data saved in config file." << std::endl;
    }

    delete port;
    return 0;
}
