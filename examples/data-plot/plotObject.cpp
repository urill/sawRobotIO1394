/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*
  $Id$

  Author(s):  Anton Deguet
  Created on: 2014-01-09

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include "plotObject.h"

plotObject::plotObject(sawRobotIO1394::osaPort1394 * port,
                       sawRobotIO1394::osaRobot1394 * robot):
    Port(port),
    Robot(robot),
    ElapsedTime(0.0),
    ActuatorIndex(5)
{
    Frame = new QFrame();
    Layout = new QVBoxLayout();

    Plot = new vctPlot2DOpenGLQtWidget();
    Layout->addWidget(Plot);

    EncoderVelocityScale = Plot->AddScale("encoder-velocities");
    EncoderDtSignal = EncoderVelocityScale->AddSignal("encoder-dt");
    EncoderDtSignal->SetColor(vct3(1.0, 0.0, 0.0));
    std::cout << "red:   encoder-dt" << std::endl;
    EncoderDxSignal = EncoderVelocityScale->AddSignal("encoder-dx");
    EncoderDxSignal->SetColor(vct3(0.0, 1.0, 0.0));
    std::cout << "green: encoder-dx" << std::endl;

    PotVelocityScale = Plot->AddScale("pot-velocities");
    PotDxSignal = PotVelocityScale->AddSignal("pot-dx");
    PotDxSignal->SetColor(vct3(1.0, 1.0, 1.0));
    std::cout << "white: pot-dx" << std::endl;

    Frame->setLayout(Layout);
    Frame->resize(1200, 600);
    Frame->show();

    PreviousTimeStamp.ForceAssign(Robot->TimeStamp());
    PreviousEncoderPosition.ForceAssign(Robot->EncoderPosition());
    PreviousPotPosition.ForceAssign(Robot->PotPosition());

    startTimer(1); // in ms
}

void plotObject::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    Port->Read();
    // get time
    ElapsedTime += Robot->TimeStamp()[ActuatorIndex];
    // encoder dt
    EncoderDtSignal->AppendPoint(vct2(ElapsedTime,
                                     Robot->EncoderVelocity()[ActuatorIndex]));
    // encoder velocity dx / dt
    EncoderDx.ForceAssign(Robot->EncoderPosition());
    EncoderDx.Subtract(PreviousEncoderPosition);
    EncoderDx.ElementwiseDivide(Robot->TimeStamp());
    EncoderDxSignal->AppendPoint((vct2(ElapsedTime,
                                       EncoderDx[ActuatorIndex])));

    // pot velocity dx / dt
    PotDx.ForceAssign(Robot->PotPosition());
    PotDx.Subtract(PreviousPotPosition);
    PotDx.ElementwiseDivide(Robot->TimeStamp());
    PotDxSignal->AppendPoint((vct2(ElapsedTime,
                                   PotDx[ActuatorIndex])));

    // update plot
    Plot->updateGL();

    // save previous state
    PreviousTimeStamp.Assign(Robot->TimeStamp());
    PreviousEncoderPosition.Assign(Robot->EncoderPosition());
    PreviousPotPosition.Assign(Robot->PotPosition());
}
