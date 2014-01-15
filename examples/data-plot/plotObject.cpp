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
#include <cisstNumerical/nmrSavitzkyGolay.h>

plotObject::plotObject(sawRobotIO1394::osaPort1394 * port,
                       sawRobotIO1394::osaRobot1394 * robot):
    Port(port),
    Robot(robot),
    ElapsedTime(0.0),
    ActuatorIndex(5),
    FilterSize(15)
{
    SavitzkyGolayCoeff = nmrSavitzkyGolay(2, // 4 degrees polynomial
                                          0, // no derivative
                                          FilterSize - 1, // nb left samples
                                          0 // nb right samples
                                          );

    History.SetSize(FilterSize);
    History.SetAll(0.0);
    FilterElementwiseProduct.SetSize(FilterSize);

    Frame = new QFrame();
    Layout = new QVBoxLayout();

    Plot = new vctPlot2DOpenGLQtWidget();
    Layout->addWidget(Plot);

    VelocityScale = Plot->AddScale("encoder-velocities");

    EncoderDtSignal = VelocityScale->AddSignal("encoder-dt");
    EncoderDtSignal->SetColor(vct3(1.0, 0.0, 0.0));
    std::cout << "red:   encoder-dt" << std::endl;

    EncoderDxSignal = VelocityScale->AddSignal("encoder-dx");
    EncoderDxSignal->SetColor(vct3(0.0, 1.0, 0.0));
    std::cout << "green: encoder-dx" << std::endl;

    EncoderDxFilteredSignal = VelocityScale->AddSignal("encoder-dx-filtered");
    EncoderDxFilteredSignal->SetColor(vct3(7.0, 7.0, 0.0));
    std::cout << "dark green: encoder-dx-filtered" << std::endl;

    PotDxSignal = VelocityScale->AddSignal("pot-dx");
    PotDxSignal->SetColor(vct3(1.0, 1.0, 1.0));
    std::cout << "white: pot-dx" << std::endl;

    ZeroVelocity = VelocityScale->AddSignal("zero");
    ZeroVelocity->SetColor(vct3(0.2, 0.2, 0.2));
    std::cout << "gray:  zero" << std::endl;

    Frame->setLayout(Layout);
    Frame->resize(1200, 600);
    Frame->show();

    PreviousEncoderPosition.ForceAssign(Robot->EncoderPosition());
    PreviousPotPosition.ForceAssign(Robot->PotPosition());

    startTimer(1); // in ms
}

void plotObject::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    Port->Read();
    // get time and plot 0 value
    ElapsedTime += Robot->TimeStamp()[ActuatorIndex];
    ZeroVelocity->AppendPoint(vct2(ElapsedTime, 0.005)); // slight offset to avoid overlap

    // encoder dt
    EncoderDtSignal->AppendPoint(vct2(ElapsedTime,
                                     Robot->EncoderVelocity()[ActuatorIndex]));
    // encoder velocity dx / dt
    EncoderDx.ForceAssign(Robot->EncoderPosition());
    EncoderDx.Subtract(PreviousEncoderPosition);
    EncoderDx.ElementwiseDivide(Robot->TimeStamp());
    EncoderDxSignal->AppendPoint((vct2(ElapsedTime,
                                       EncoderDx[ActuatorIndex])));

    // filtered dx / dt
    // update history
    for (size_t index = 1; index < FilterSize; ++index) {
        History[index - 1] = History[index];
    }
    History[FilterSize - 1] = EncoderDx[ActuatorIndex];
    // apply filter
    FilterElementwiseProduct.ElementwiseProductOf(SavitzkyGolayCoeff, History);
    EncoderDxFilteredSignal->AppendPoint(vct2(ElapsedTime, FilterElementwiseProduct.SumOfElements()));

    // pot velocity dx / dt
    PotDx.ForceAssign(Robot->PotPosition());
    PotDx.Subtract(PreviousPotPosition);
    PotDx.ElementwiseDivide(Robot->TimeStamp());
    PotDxSignal->AppendPoint((vct2(ElapsedTime,
                                   PotDx[ActuatorIndex])));

    // update plot
    Plot->updateGL();

    // save previous state
    PreviousEncoderPosition.Assign(Robot->EncoderPosition());
    PreviousPotPosition.Assign(Robot->PotPosition());
}
