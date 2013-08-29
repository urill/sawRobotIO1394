/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen
  Created on: 2013-02-16

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// Qt include
#include <QString>
#include <QtGui>

// project include
#include <sawRobotIO1394/mtsRobotIO1394QtWidget.h>

#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmJointType.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsRobotIO1394QtWidget, mtsComponent, mtsComponentConstructorNameAndUInt)


mtsRobotIO1394QtWidget::mtsRobotIO1394QtWidget(const std::string & componentName, unsigned int numberOfActuators):
    mtsComponent(componentName),
    DirectControl(false),
    TimerPeriodInMilliseconds(50),
    NumberOfActuators(numberOfActuators)
{
    WatchdogPeriodInSeconds = 300.0 * cmn_ms;
    Init();
}

mtsRobotIO1394QtWidget::mtsRobotIO1394QtWidget(const mtsComponentConstructorNameAndUInt &arg):
    mtsComponent(arg.Name), NumberOfActuators(arg.Arg)
{
    Init();
}

void mtsRobotIO1394QtWidget::Init(void)
{
    DummyValueWhenNotConnected = 0;
    LastEnableState.SetSize(NumberOfActuators);
    LastEnableState.SetAll(false);

    UnitFactor.SetSize(NumberOfActuators);
    JointPosition.SetSize(NumberOfActuators);
    ActuatorPosition.SetSize(NumberOfActuators);
    ActuatorPositionGet.Position().SetSize(NumberOfActuators);
    ActuatorVelocity.SetSize(NumberOfActuators);
    PotentiometersVolts.SetSize(NumberOfActuators);
    PotentiometersPosition.SetSize(NumberOfActuators);
    MotorFeedbackCurrent.SetSize(NumberOfActuators);
    MotorFeedbackCurrent.Zeros();
    MotorControlCurrent.SetSize(NumberOfActuators);
    MotorControlCurrent.Zeros();
    AmpEnable.SetSize(NumberOfActuators);
    AmpTemperature.SetSize(NumberOfActuators);

    StartTime = osaGetTime();

    SetupCisstInterface();
    setupUi();

    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsRobotIO1394QtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsRobotIO1394QtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    vctDoubleVec motorCurrentMax(this->NumberOfActuators);
    mtsExecutionResult result = Robot.GetMotorCurrentMax(motorCurrentMax);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get motor current max.  Function call returned: "
                                 << result << std::endl;
    } else {
        // convert to mA
        motorCurrentMax.Multiply(1000.0);
        QVWCurrentSpinBoxWidget->SetRange(-motorCurrentMax, motorCurrentMax);
        QVWCurrentSliderWidget->SetRange(-motorCurrentMax, motorCurrentMax);
    }

    prmJointTypeVec jointType;
    result = Robot.GetJointType(jointType);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                 << result << std::endl;
        UnitFactor.SetAll(0.0);
    } else {
        // set unitFactor;
        for (size_t i = 0; i < this->NumberOfActuators; i++ ){
            if (jointType[i] == PRM_REVOLUTE)
                UnitFactor[i] = cmn180_PI;
            else if (jointType[i] == PRM_PRISMATIC)
                UnitFactor[i] = cmn_mm;
            else
                cmnThrow("mtsRobotIO1394QtWidget: Unknown joint type");
        }
    }
    if (!parent()) {
        show();
    }
}

void mtsRobotIO1394QtWidget::Cleanup(void)
{
    this->hide();
    Robot.DisablePower();
    Robot.DisableSafetyRelay();
    Actuators.DisableBoardsPower();
}

void mtsRobotIO1394QtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsRobotIO1394QtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsRobotIO1394QtWidget::SlotEnableAmps(bool toggle)
{
    // send to controller first
    if (toggle) {
        Actuators.EnableBoardsPower();
    } else {
        Actuators.DisableBoardsPower();
    }
    // update GUI, make sure no signal is generated
    if (!toggle) {
        QCBEnableAll->blockSignals(true);
        {
            QCBEnableAll->setChecked(false);
        }
        QCBEnableAll->blockSignals(false);
    }
    // set all current to 0
    SlotResetCurrentAll();
}

void mtsRobotIO1394QtWidget::SlotEnableAll(bool toggle)
{
    // send to controller first
    if (toggle) {
        Robot.EnablePower();
    } else {
        Robot.DisablePower();
    }
    // update GUI, make sure no signal is generated
    QCBEnableAmps->blockSignals(true);
    {
        QCBEnableAmps->setChecked(toggle);
    } QCBEnableAmps->blockSignals(false);
    vctBoolVec allEnable(NumberOfActuators, toggle);
    QVWCurrentEnableEachWidget->SetValue(allEnable);
    // set all current to 0
    SlotResetCurrentAll();
}

void mtsRobotIO1394QtWidget::SlotEnableDirectControl(bool toggle)
{
    DirectControl = toggle;
    // if checked in DIRECT_CONTROL mode
    QVWCurrentSpinBoxWidget->setEnabled(toggle);
    QVWCurrentSliderWidget->setEnabled(toggle);
    QPBResetCurrentAll->setEnabled(toggle);
    QPBBiasCurrentAll->setEnabled(toggle);
    // set all current to 0
    SlotResetCurrentAll();
}

void mtsRobotIO1394QtWidget::SlotResetCurrentAll(void)
{
    // send to controller first
    vctDoubleVec cmdCurA(NumberOfActuators);
    cmdCurA.SetAll(0.0);
    Robot.SetMotorCurrent(cmdCurA);
    // update GUI
    QVWCurrentSpinBoxWidget->SetValue(cmdCurA);
    QVWCurrentSliderWidget->SetValue(cmdCurA);
}

void mtsRobotIO1394QtWidget::SlotBiasCurrentAll(void)
{
    mtsExecutionResult result = Robot.BiasCurrent(mtsInt(1000)); // use a 1000 samples to average current feedback
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "slot_qpbBiasCurrentAll: command failed \"" << result << "\"" << std::endl;
    }
}

void mtsRobotIO1394QtWidget::SlotEnable(void)
{
    AmpEnable.SetSize(NumberOfActuators);
    QVWCurrentEnableEachWidget->GetValue(AmpEnable);
    Actuators.SetAmpEnable(AmpEnable);
}

void mtsRobotIO1394QtWidget::SlotMotorCurrentValueChanged()
{
    vctDoubleVec cmdCurmA(NumberOfActuators);
    vctDoubleVec cmdCurA(NumberOfActuators);
    // get value from GUI
    QVWCurrentSpinBoxWidget->GetValue(cmdCurmA);
    QVWCurrentSliderWidget->SetValue(cmdCurmA);
    // convert to amps and apply
    cmdCurA = cmdCurmA.Divide(1000.0);
    Robot.SetMotorCurrent(cmdCurA);
}

void mtsRobotIO1394QtWidget::SlotSliderMotorCurrentValueChanged()
{
    vctDoubleVec cmdCurmA(NumberOfActuators);
    vctDoubleVec cmdCurA(NumberOfActuators);
    // get value from GUI
    QVWCurrentSliderWidget->GetValue(cmdCurmA);
    QVWCurrentSpinBoxWidget->SetValue(cmdCurmA);
    // convert to amps and apply
    cmdCurA = cmdCurmA.Divide(1000.0);
    Robot.SetMotorCurrent(cmdCurA);
}

void mtsRobotIO1394QtWidget::SlotResetEncodersAll()
{
    vctDoubleVec newEncoderValues(NumberOfActuators);
    newEncoderValues.SetAll(0.0);
    mtsExecutionResult result = Robot.SetEncoderPosition(newEncoderValues);
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "slot_qpbResetEncAll: command failed \"" << result << "\"" << std::endl;
     }
}

void mtsRobotIO1394QtWidget::SlotBiasEncodersAll()
{
    mtsExecutionResult result = Robot.BiasEncoder();
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "slot_qpbBiasEncAll: command failed \"" << result << "\"" << std::endl;
    }
}

void mtsRobotIO1394QtWidget::SlotWatchdogPeriod(double period_ms)
{
    if (period_ms == 0.0) {
        QMessageBox message;
        message.setText("Setting the watchdog period to 0 disables the watchdog!");
        message.exec();
    }
    WatchdogPeriodInSeconds = period_ms * cmn_ms;
    mtsExecutionResult result = Robot.SetWatchdogPeriod(WatchdogPeriodInSeconds);
    if(!result.IsOK()){
        CMN_LOG_CLASS_RUN_WARNING << "slot_qdsbWatchdogPeriod: command failed \""
                                  << result << "\"" << std::endl;
    }
}

void mtsRobotIO1394QtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    ProcessQueuedEvents();

    bool flag;
    Robot.IsValid(flag);
    if (flag) {
        Robot.GetPeriodStatistics(IntervalStatistics);
        Robot.GetPosition(JointPosition); // vct
        JointPosition.ElementwiseMultiply(UnitFactor); // to degrees or mm
        Actuators.GetPositionActuator(ActuatorPositionGet); // prm
        ActuatorPosition.Assign(ActuatorPositionGet.Position()); // vct
        ActuatorPosition.ElementwiseMultiply(UnitFactor); // to degrees or mm
        Robot.GetVelocity(ActuatorVelocity);
        ActuatorVelocity.ElementwiseMultiply(UnitFactor); // to degrees or mm
        Robot.GetAnalogInputVolts(PotentiometersVolts);
        Robot.GetAnalogInputPosSI(PotentiometersPosition);
        PotentiometersPosition.Multiply((cmn180_PI));
        Robot.GetMotorFeedbackCurrent(MotorFeedbackCurrent);
        Actuators.GetAmpEnable(AmpEnable);
        Actuators.GetAmpStatus(AmpStatus);
        Robot.GetPowerStatus(PowerStatus);
        Robot.GetSafetyRelay(SafetyRelay);
        Robot.GetAmpTemperature(AmpTemperature);
    } else {
        JointPosition.SetAll(DummyValueWhenNotConnected);
        ActuatorPosition.SetAll(DummyValueWhenNotConnected);
        ActuatorVelocity.SetAll(DummyValueWhenNotConnected);
        PotentiometersVolts.SetAll(DummyValueWhenNotConnected);
        PotentiometersPosition.SetAll(DummyValueWhenNotConnected);
        MotorFeedbackCurrent.SetAll(DummyValueWhenNotConnected);
        AmpTemperature.SetAll(DummyValueWhenNotConnected);
    }

    CMN_LOG_CLASS_RUN_VERBOSE << (osaGetTime() - StartTime) << std::endl;

    DummyValueWhenNotConnected += 0.1;

    // display requested current when we are not trying to set it using GUI
    if (flag && !DirectControl) {
        vctDoubleVec requestedCurrent(NumberOfActuators);
        Robot.GetMotorRequestedCurrent(requestedCurrent);
        requestedCurrent.Multiply(1000.0); // got A, need mA for display
        QVWCurrentSpinBoxWidget->SetValue(requestedCurrent);
        QVWCurrentSliderWidget->SetValue(requestedCurrent);
    }

    QMIntervalStatistics->SetValue(IntervalStatistics);
    QVRJointPositionWidget->SetValue(JointPosition);
    QVRActuatorPositionWidget->SetValue(ActuatorPosition);
    QVRActuatorVelocityWidget->SetValue(ActuatorVelocity);
    QVRPotVoltsWidget->SetValue(PotentiometersVolts);
    QVRPotPositionWidget->SetValue(PotentiometersPosition);
    QVRCurrentFeedbackWidget->SetValue(MotorFeedbackCurrent * 1000.0);
    QVRAmpTemperature->SetValue(AmpTemperature);

    UpdateRobotInfo();

    Robot.SetWatchdogPeriod(WatchdogPeriodInSeconds);
}

////------------ Private Methods ----------------
void mtsRobotIO1394QtWidget::SetupCisstInterface(void)
{
    // Required Interface
    mtsInterfaceRequired * robotInterface = AddInterfaceRequired("Robot");
    if (robotInterface) {
        robotInterface->AddFunction("GetPeriodStatistics", Robot.GetPeriodStatistics);
        robotInterface->AddFunction("GetNumberOfActuators", Robot.GetNumberOfActuators);
        robotInterface->AddFunction("IsValid", Robot.IsValid);
        robotInterface->AddFunction("EnablePower", Robot.EnablePower);
        robotInterface->AddFunction("DisablePower", Robot.DisablePower);
        robotInterface->AddFunction("EnableSafetyRelay", Robot.EnableSafetyRelay);
        robotInterface->AddFunction("DisableSafetyRelay", Robot.DisableSafetyRelay);

        robotInterface->AddFunction("GetPosition", Robot.GetPosition);
        robotInterface->AddFunction("GetVelocity", Robot.GetVelocity);
        robotInterface->AddFunction("GetAnalogInputVolts", Robot.GetAnalogInputVolts);
        robotInterface->AddFunction("GetAnalogInputPosSI", Robot.GetAnalogInputPosSI);
        robotInterface->AddFunction("GetMotorRequestedCurrent", Robot.GetMotorRequestedCurrent);
        robotInterface->AddFunction("GetMotorFeedbackCurrent", Robot.GetMotorFeedbackCurrent);
        robotInterface->AddFunction("GetMotorCurrentMax", Robot.GetMotorCurrentMax);
        robotInterface->AddFunction("GetJointType", Robot.GetJointType);
        robotInterface->AddFunction("GetPowerStatus", Robot.GetPowerStatus);
        robotInterface->AddFunction("GetSafetyRelay", Robot.GetSafetyRelay);
        robotInterface->AddFunction("GetAmpTemperature", Robot.GetAmpTemperature);

        robotInterface->AddFunction("SetMotorCurrent", Robot.SetMotorCurrent);
        robotInterface->AddFunction("SetEncoderPosition", Robot.SetEncoderPosition);
        robotInterface->AddFunction("SetWatchdogPeriod", Robot.SetWatchdogPeriod);

        robotInterface->AddFunction("BiasCurrent", Robot.BiasCurrent);
        robotInterface->AddFunction("BiasEncoder", Robot.BiasEncoder);

        // make sure the events are queued
        robotInterface->AddEventHandlerWrite(&mtsRobotIO1394QtWidget::PowerStatusEventHandler, this, "PowerStatus");
        robotInterface->AddEventHandlerWrite(&mtsRobotIO1394QtWidget::WatchdogStatusEventHandler, this, "WatchdogStatus");
    }

    mtsInterfaceRequired * actuatorInterface = AddInterfaceRequired("RobotActuators");
    if (actuatorInterface) {
        actuatorInterface->AddFunction("GetPositionActuator", Actuators.GetPositionActuator);

        actuatorInterface->AddFunction("EnableBoardsPower", Actuators.EnableBoardsPower);
        actuatorInterface->AddFunction("DisableBoardsPower", Actuators.DisableBoardsPower);
        actuatorInterface->AddFunction("SetAmpEnable", Actuators.SetAmpEnable);
        actuatorInterface->AddFunction("ResetSingleEncoder", Actuators.ResetSingleEncoder);

        actuatorInterface->AddFunction("GetAmpEnable", Actuators.GetAmpEnable);
        actuatorInterface->AddFunction("GetAmpStatus", Actuators.GetAmpStatus);
    }
}



void mtsRobotIO1394QtWidget::setupUi(void)
{
    QFont font;
    font.setBold(true);

    // Power commands
    QVBoxLayout * powerLayout = new QVBoxLayout;
    QFrame * powerFrame = new QFrame;
    QLabel * powerTitle = new QLabel("Power");
    powerTitle->setFont(font);
    powerTitle->setAlignment(Qt::AlignCenter);
    powerLayout->addWidget(powerTitle);
    QCBEnableAll = new QCheckBox("Enable all");
    powerLayout->addWidget(QCBEnableAll);
    QCBEnableAmps = new QCheckBox("Enable boards");
    powerLayout->addWidget(QCBEnableAmps);
    powerLayout->addSpacing(5);
    QLAmpStatus = new QLabel("Actuators ON");
    QLAmpStatus->setAlignment(Qt::AlignCenter);
    powerLayout->addWidget(QLAmpStatus);
    QLPowerStatus = new QLabel("Boards ON");
    QLPowerStatus->setAlignment(Qt::AlignCenter);
    powerLayout->addWidget(QLPowerStatus);
    powerLayout->addStretch();
    powerFrame->setLayout(powerLayout);
    powerFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // watchdog commands
    QVBoxLayout * watchdogLayout = new QVBoxLayout;
    QFrame * watchdogFrame = new QFrame;
    QLabel * watchdogTitle = new QLabel("Watchdog");
    watchdogTitle->setFont(font);
    watchdogTitle->setAlignment(Qt::AlignCenter);
    watchdogLayout->addWidget(watchdogTitle);
    QHBoxLayout * watchdogSetLayout = new QHBoxLayout;
    {
        QLabel * wdogLabel = new QLabel("Wdog period (ms)");
        QSBWatchdogPeriod = new QDoubleSpinBox;
        QSBWatchdogPeriod->setMaximum(340.0); // max wdog_period = 340 ms
        QSBWatchdogPeriod->setMinimum(0.0);
        QSBWatchdogPeriod->setSingleStep(0.05);
        QSBWatchdogPeriod->setValue(cmnInternalTo_ms(WatchdogPeriodInSeconds));
        watchdogSetLayout->addWidget(wdogLabel);
        watchdogSetLayout->addWidget(QSBWatchdogPeriod);
    }
    watchdogLayout->addLayout(watchdogSetLayout);
    watchdogLayout->addSpacing(5);
    QLSafetyRelay = new QLabel("Safety relay ON");
    QLSafetyRelay->setAlignment(Qt::AlignCenter);
    watchdogLayout->addWidget(QLSafetyRelay);
    QLWatchdog = new QLabel("Watchdog Timeout FALSE");
    QLWatchdog->setAlignment(Qt::AlignCenter);
    QLWatchdog->setStyleSheet("QLabel { background-color: green }");
    watchdogLayout->addWidget(QLWatchdog);
    QLWatchdogLastTimeout = new QLabel("Last timeout: n/a");
    QLWatchdogLastTimeout->setAlignment(Qt::AlignCenter);
    watchdogLayout->addWidget(QLWatchdogLastTimeout);
    watchdogLayout->addStretch();
    watchdogFrame->setLayout(watchdogLayout);
    watchdogFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Encoder commands
    QVBoxLayout * encoderLayout = new QVBoxLayout;
    QFrame * encoderFrame = new QFrame;
    QLabel * encoderTitle = new QLabel("Encoders");
    encoderTitle->setFont(font);
    encoderTitle->setAlignment(Qt::AlignCenter);
    encoderLayout->addWidget(encoderTitle);
    QPBResetEncAll = new QPushButton("Reset all");
    encoderLayout->addWidget(QPBResetEncAll);
    QPBBiasEncAll = new QPushButton("Bias from potentiometers");
    encoderLayout->addWidget(QPBBiasEncAll);
    encoderLayout->addStretch();
    encoderFrame->setLayout(encoderLayout);
    encoderFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Current comands
    QVBoxLayout * currentLayout = new QVBoxLayout;
    QFrame * currentFrame = new QFrame;
    QLabel * currentTitle = new QLabel("Current");
    currentTitle->setFont(font);
    currentTitle->setAlignment(Qt::AlignCenter);
    currentLayout->addWidget(currentTitle);
    QCBEnableDirectControl = new QCheckBox("Direct control");
    currentLayout->addWidget(QCBEnableDirectControl);
    QPBBiasCurrentAll = new QPushButton("Bias from feedback");
    currentLayout->addWidget(QPBBiasCurrentAll);
    currentLayout->addStretch();
    currentFrame->setLayout(currentLayout);
    currentFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Timing
    QVBoxLayout * timingLayout = new QVBoxLayout;
    QFrame * timingFrame = new QFrame;
    QLabel * timingTitle = new QLabel("Timing");
    timingTitle->setFont(font);
    timingTitle->setAlignment(Qt::AlignCenter);
    timingLayout->addWidget(timingTitle);
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    timingLayout->addWidget(QMIntervalStatistics);
    timingLayout->addStretch();
    timingFrame->setLayout(timingLayout);
    timingFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Commands layout
    QHBoxLayout * commandLayout = new QHBoxLayout;
    commandLayout->addWidget(powerFrame);
    commandLayout->addWidget(watchdogFrame);
    commandLayout->addWidget(encoderFrame);
    commandLayout->addWidget(currentFrame);
    commandLayout->addWidget(timingFrame);

    // Feedbacks Label
    QGridLayout * gridLayout = new QGridLayout;
    gridLayout->setSpacing(1);
    int row = 0;

    vctBoolVec defaultEnable(NumberOfActuators, false);
    vctDoubleVec defaultCurrent(NumberOfActuators, 0.0);

    gridLayout->addWidget(new QLabel("Actuator power"), row, 0);
    QVWCurrentEnableEachWidget = new vctQtWidgetDynamicVectorBoolWrite();
    QVWCurrentEnableEachWidget->SetValue(defaultEnable);
    gridLayout->addWidget(QVWCurrentEnableEachWidget, row, 1);
    row++;

    gridLayout->addWidget(new QLabel("Desired current (mA)"), row, 0);
    QVWCurrentSpinBoxWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWCurrentSpinBoxWidget->SetValue(defaultCurrent);
    gridLayout->addWidget(QVWCurrentSpinBoxWidget, row, 1);
    row++;

    QPBResetCurrentAll = new QPushButton("Reset current");
    gridLayout->addWidget(QPBResetCurrentAll, row, 0);
    QVWCurrentSliderWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SLIDER_WIDGET);
    QVWCurrentSliderWidget->SetValue(defaultCurrent);
    gridLayout->addWidget(QVWCurrentSliderWidget, row, 1);
    row++;

    gridLayout->addWidget(new QLabel("Joint positions (deg)"), row, 0);
    QVRJointPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(QVRJointPositionWidget, row, 1);
    row++;

    gridLayout->addWidget(new QLabel("Actuator positions (deg)"), row, 0);
    QVRActuatorPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(QVRActuatorPositionWidget, row, 1);
    row++;

    gridLayout->addWidget(new QLabel("Velocities (deg/s)"), row, 0);
    QVRActuatorVelocityWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(QVRActuatorVelocityWidget, row, 1);
    row++;

    gridLayout->addWidget(new QLabel("Analog inputs (V)"), row, 0);
    QVRPotVoltsWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(QVRPotVoltsWidget, row, 1);
    row++;

    gridLayout->addWidget(new QLabel("Potentiometers (deg)"), row, 0);
    QVRPotPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(QVRPotPositionWidget, row, 1);
    row++;

    gridLayout->addWidget(new QLabel("Current feedback (mA)"), row, 0);
    QVRCurrentFeedbackWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(QVRCurrentFeedbackWidget, row, 1);
    row++;

    gridLayout->addWidget(new QLabel("Amp temperature (C)"), row, 0);
    QVRAmpTemperature = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(QVRAmpTemperature, row, 1);
    row++;

    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addLayout(commandLayout);
    mainLayout->addLayout(gridLayout);
    mainLayout->addStretch();

    setLayout(mainLayout);

    setWindowTitle(QString(this->GetName().c_str()));
    resize(sizeHint());

    // connect signals & slots
    connect(QCBEnableAmps, SIGNAL(toggled(bool)), this, SLOT(SlotEnableAmps(bool)));
    connect(QCBEnableAll, SIGNAL(toggled(bool)), this, SLOT(SlotEnableAll(bool)));
    connect(QCBEnableDirectControl, SIGNAL(toggled(bool)), this, SLOT(SlotEnableDirectControl(bool)));
    connect(QPBResetCurrentAll, SIGNAL(clicked()), this, SLOT(SlotResetCurrentAll()));
    connect(QPBBiasCurrentAll, SIGNAL(clicked()), this, SLOT(SlotBiasCurrentAll()));
    connect(QPBResetEncAll, SIGNAL(clicked()), this, SLOT(SlotResetEncodersAll()));
    connect(QPBBiasEncAll, SIGNAL(clicked()), this, SLOT(SlotBiasEncodersAll()));
    connect(QSBWatchdogPeriod, SIGNAL(valueChanged(double)),
            this, SLOT(SlotWatchdogPeriod(double)));
    connect(QVWCurrentEnableEachWidget, SIGNAL(valueChanged()), this, SLOT(SlotEnable()));
    connect(QVWCurrentSpinBoxWidget, SIGNAL(valueChanged()), this, SLOT(SlotMotorCurrentValueChanged()));
    connect(QVWCurrentSliderWidget, SIGNAL(valueChanged()), this, SLOT(SlotSliderMotorCurrentValueChanged()));

    // connect cisstMultiTask events
    connect(this, SIGNAL(SignalPowerStatus(bool)), this, SLOT(SlotPowerStatus(bool)));
    connect(this, SIGNAL(SignalWatchdogStatus(bool)), this, SLOT(SlotWatchdogStatus(bool)));

    // set initial value
    QCBEnableAmps->setChecked(false);
    QCBEnableAll->setChecked(false);
    QCBEnableDirectControl->setChecked(DirectControl);
    SlotEnableDirectControl(DirectControl);
}


void mtsRobotIO1394QtWidget::UpdateRobotInfo(void)
{
    // amplifier status
    bool ampStatusGood = AmpStatus.All();
    QVWCurrentEnableEachWidget->SetValue(AmpStatus);
    if (ampStatusGood) {
        QLAmpStatus->setText("Actuators ON");
        QLAmpStatus->setStyleSheet("QPLabel { background-color: green }");
    } else {
        QLAmpStatus->setText("Actuators OFF");
        QLAmpStatus->setStyleSheet("QLabel { background-color: red }");
    }

    // power status
    if (PowerStatus) {
        QLPowerStatus->setText("Boards ON");
        QLPowerStatus->setStyleSheet("QLabel { background-color: green }");
    } else {
        QLPowerStatus->setText("Boards OFF");
        QLPowerStatus->setStyleSheet("QLabel { background-color: red }");
    }

    // update check box to enable/disable based on current state
    QCBEnableAmps->blockSignals(true); {
        QCBEnableAmps->setChecked(ampStatusGood);
    } QCBEnableAmps->blockSignals(false);
    QCBEnableAll->blockSignals(true); {
        QCBEnableAll->setChecked(ampStatusGood && AmpEnable.All());
    } QCBEnableAll->blockSignals(false);

    // safety Relay
    if (SafetyRelay) {
        QLSafetyRelay->setText("Safety Relay ON");
        QLSafetyRelay->setStyleSheet("QLabel { background-color: green }");
    } else {
        QLSafetyRelay->setText("Safety Relay OFF");
        QLSafetyRelay->setStyleSheet("QLabel { background-color: red }");
    }
}

void mtsRobotIO1394QtWidget::PowerStatusEventHandler(const bool & status)
{
    emit SignalPowerStatus(status);
}

void mtsRobotIO1394QtWidget::SlotPowerStatus(bool status)
{
    if (status == false) {
        QCBEnableAll->setChecked(false);
    }
}

void mtsRobotIO1394QtWidget::WatchdogStatusEventHandler(const bool & status)
{
    emit SignalWatchdogStatus(status);
}

void mtsRobotIO1394QtWidget::SlotWatchdogStatus(bool status)
{
    if (status) {
        QLWatchdog->setText("Watchdog Timeout TRUE");
        QLWatchdog->setStyleSheet("QLabel { background-color: red }");
        QLWatchdogLastTimeout->setText(QString("Last timeout: " + QTime::currentTime().toString("hh:mm:ss")));
    } else {
        QLWatchdog->setText("Watchdog Timeout FALSE");
        QLWatchdog->setStyleSheet("QLabel { background-color: green }");
    }
}
