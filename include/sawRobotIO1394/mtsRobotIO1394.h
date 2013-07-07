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

#ifndef _mtsRobotIO1394_h
#define _mtsRobotIO1394_h

#include <ostream>
#include <iostream>
#include <vector>

#include <cisstMultiTask/mtsTaskPeriodic.h>

namespace sawRobotIO1394 {
    class osaIO1394Port;
    class mtsIO1394Robot;
    class mtsIO1394DigitalInput;
}

class mtsRobotIO1394 : public mtsTaskPeriodic {

    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);
public:
    enum { MAX_BOARDS = 16 };

protected:

    std::ostream * MessageStream;             // Stream provided to the low level boards for messages, redirected to cmnLogger

    sawRobotIO1394::osaIO1394Port * Port_;

    typedef std::vector<sawRobotIO1394::mtsIO1394Robot*> RobotsType;
    typedef typename RobotsType::iterator robots_iterator;
    RobotsType Robots_;

    typedef std::vector<sawRobotIO1394::mtsIO1394DigitalInput*> DigitalInputsType;
    typedef typename DigitalInputsType::iterator digital_inputs_iterator;
    DigitalInputsType DigitalInputs_;

    ///////////// Public Class Methods ///////////////////////////
public:
    // Constructor & Destructor
    mtsRobotIO1394(const std::string & name, double periodInSeconds, int portNumber);
    mtsRobotIO1394(const mtsTaskPeriodicConstructorArg & arg); // TODO: add port_num
    virtual ~mtsRobotIO1394();

    void Init(int portNumber);

    void Configure(const std::string & filename);
    bool SetupRobot(sawRobotIO1394::mtsIO1394Robot * robot);
    bool SetupDigitalInput(sawRobotIO1394::mtsIO1394DigitalInput * digitalInput);
    void Startup(void);
    void Run(void);
    void Cleanup(void);
    void GetNumberOfDigitalInputs(int & placeHolder) const;

protected:

    void GetNumberOfBoards(int & placeHolder) const;
    void GetNumberOfRobots(int & placeHolder) const;

    void GetNumberOfActuatorPerRobot(vctIntVec & placeHolder) const;
    void GetName(std::string & placeHolder) const;

    void PreRead(void);
    void PostRead(void);

    ////////////// Private Class Methods /////////////////////////////
private:
    // Make uncopyable
    mtsRobotIO1394(const mtsRobotIO1394 &);
    mtsRobotIO1394 &operator=(const mtsRobotIO1394 &);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobotIO1394);

#endif // _mtsRobotIO1394_h
