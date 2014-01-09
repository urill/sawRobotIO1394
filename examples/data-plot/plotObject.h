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

#ifndef _plotObject_h
#define _plotObject_h

// system
#include <iostream>

// cisst/saw
#include <sawRobotIO1394/osaPort1394.h>
#include <sawRobotIO1394/osaRobot1394.h>

// Qt
#include <QFrame>
#include <QVBoxLayout>
#include <cisstVector/vctPlot2DOpenGLQtWidget.h>

class plotObject: public QObject
{
    Q_OBJECT;
public:
    plotObject(sawRobotIO1394::osaPort1394 * port,
               sawRobotIO1394::osaRobot1394 * robot);

private slots:
    void timerEvent(QTimerEvent * CMN_UNUSED(event));

protected:
    QFrame * Frame;
    QVBoxLayout * Layout;
    vctPlot2DOpenGLQtWidget * Plot;
    vctPlot2DBase::Scale * EncoderVelocityScale;
    vctPlot2DBase::Signal * EncoderDtSignal;
    vctPlot2DBase::Signal * EncoderDxSignal;
    vctPlot2DBase::Scale * PotVelocityScale;
    vctPlot2DBase::Signal * PotDxSignal;

    sawRobotIO1394::osaPort1394 * Port;
    sawRobotIO1394::osaRobot1394 * Robot;

    double ElapsedTime;
    int ActuatorIndex;
    vctDoubleVec PreviousTimeStamp;
    vctDoubleVec PreviousEncoderPosition;
    vctDoubleVec EncoderDx;
    vctDoubleVec PreviousPotPosition;
    vctDoubleVec PotDx;

protected slots:
    //    void DoubleTextValueChangedSlot(void);
};

#endif // _plotObject_h
