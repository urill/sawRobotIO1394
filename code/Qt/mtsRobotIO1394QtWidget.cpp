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

#define SWITCH 1

// Find a better way for user to get these values
#define MOTORCUR_MAX 6250       // max motor current in mA
#define MOTORCUR_DAC 0xFFFF     // max dac value for motor current


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsRobotIO1394QtWidget, mtsComponent, std::string);


mtsRobotIO1394QtWidget::mtsRobotIO1394QtWidget(const std::string & taskName, unsigned int numberOfActuators):
    mtsComponent(taskName), numOfAxis(numberOfActuators)
{
    Init();
}

mtsRobotIO1394QtWidget::mtsRobotIO1394QtWidget(const std::string & taskName):
    mtsComponent(taskName), numOfAxis(8)
{
    Init();
}

void mtsRobotIO1394QtWidget::Init(void)
{
    curFBState = false;
    curFBPGain = 1.0;
    curFBOffset = 30.0;
    tmpStatic = 0;
    lastEnableState.SetSize(numOfAxis);
    lastEnableState.SetAll(false);

    jointPos.SetSize(numOfAxis);
    actuatorPos.SetSize(numOfAxis);
    actuatorPosGet.Position().SetSize(numOfAxis);
    vel.SetSize(numOfAxis);
    potVolt.SetSize(numOfAxis);
    potPosSI.SetSize(numOfAxis);
    motorFeedbackCurrent.SetSize(numOfAxis);
    motorFeedbackCurrent.SetAll(0.0);
    motorControlCurrent.SetSize(numOfAxis);
    motorControlCurrent.SetAll(0.0);
    ampEnable.SetSize(numOfAxis);

    startTime = osaGetTime();

    setupMenu();
    setupCisstInterface();
    setupUi();

    startTimer(50); // ms
}

void mtsRobotIO1394QtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsRobotIO1394QtWidget::Startup()
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    show();
}

void mtsRobotIO1394QtWidget::Cleanup()
{
    Robot.DisablePower();
    Robot.DisableSafetyRelay();
}


//---------- Protected --------------------------
void mtsRobotIO1394QtWidget::closeEvent(QCloseEvent * event)
{
    int ret = QMessageBox::warning(this, tr("ExitBox"),
                                 tr("Please power off the robot before quit. \n"
                                    "Continue?"),
                                 QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    if(ret == QMessageBox::Yes) {
        Robot.DisablePower();
        event->accept();
    }else {
        event->ignore();
    }
}


//----------- Private Slot ------------------------------
void mtsRobotIO1394QtWidget::slot_qcbEnableBoards(bool toggle)
{
    // send to controller first
    if (toggle) {
        Actuators.EnableBoardsPower();
    } else {
        Actuators.DisableBoardsPower();
    }
    // update GUI, make sure no signal is generated
    if (!toggle) {
        qcbEnableAll->blockSignals(true); {
            qcbEnableAll->setChecked(false);
        } qcbEnableAll->blockSignals(false);
    }
}

void mtsRobotIO1394QtWidget::slot_qcbEnableAll(bool toggle)
{
    // send to controller first
    if (toggle) {
        Robot.EnablePower();
    } else {
        Robot.DisablePower();
    }
    // update GUI, make sure no signal is generated
    qcbEnableBoards->blockSignals(true); {
        qcbEnableBoards->setChecked(toggle);
    } qcbEnableBoards->blockSignals(false);
    vctBoolVec allEnable(numOfAxis, toggle);
    CurrentEnableEachWidget->SetValue(allEnable);
}

void mtsRobotIO1394QtWidget::slot_qcbEnableDirectControl(bool toggle)
{
    // if checked in DIRECT_CONTROL mode
    CurrentSpinBoxWidget->setEnabled(toggle);
    CurrentSliderWidget->setEnabled(toggle);
    qpbResetCurrentAll->setEnabled(toggle);
    qpbBiasCurrentAll->setEnabled(toggle);
    // set all current to 0
    slot_qpbResetCurrentAll();
}

void mtsRobotIO1394QtWidget::slot_qpbResetCurrentAll(void)
{
    // send to controller first
    vctDoubleVec cmdCurA(numOfAxis);
    cmdCurA.SetAll(0.0);
    Robot.SetMotorCurrent(cmdCurA);
    // update GUI
    CurrentSpinBoxWidget->SetValue(cmdCurA);
    CurrentSliderWidget->SetValue(cmdCurA);
}

void mtsRobotIO1394QtWidget::slot_qpbBiasCurrentAll(void)
{
    mtsExecutionResult result = Robot.BiasCurrent(mtsInt(1000)); // use a 1000 samples to average current feedback
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "slot_qpbBiasCurrentAll: command failed \"" << result << "\"" << std::endl;
    }
}

void mtsRobotIO1394QtWidget::slot_qcbEnable(void)
{
    ampEnable.SetSize(numOfAxis);
    CurrentEnableEachWidget->GetValue(ampEnable);
    Actuators.SetAmpEnable(ampEnable);
}

void mtsRobotIO1394QtWidget::slot_qdsbMotorCurrent_valueChanged()
{
    vctDoubleVec cmdCurmA(numOfAxis);
    vctDoubleVec cmdCurA(numOfAxis);
    // get value from GUI
    CurrentSpinBoxWidget->GetValue(cmdCurmA);
    CurrentSliderWidget->SetValue(cmdCurmA);
    // convert to amps and apply
    cmdCurA = cmdCurmA.Divide(1000.0);
    Robot.SetMotorCurrent(cmdCurA);
}

void mtsRobotIO1394QtWidget::slot_qsliderMotorCurrent_valueChanged()
{
    vctDoubleVec cmdCurmA(numOfAxis);
    vctDoubleVec cmdCurA(numOfAxis);
    // get value from GUI
    CurrentSliderWidget->GetValue(cmdCurmA);
    CurrentSpinBoxWidget->SetValue(cmdCurmA);
    // convert to amps and apply
    cmdCurA = cmdCurmA.Divide(1000.0);
    Robot.SetMotorCurrent(cmdCurA);
}

void mtsRobotIO1394QtWidget::slot_qpbResetEncAll()
{
    vctDoubleVec newEncoderValues(numOfAxis);
    newEncoderValues.SetAll(0.0);
    mtsExecutionResult result = Robot.SetEncoderPosition(newEncoderValues);
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "slot_qpbResetEncAll: command failed \"" << result << "\"" << std::endl;
     }
}

void mtsRobotIO1394QtWidget::slot_qpbBiasEncAll()
{
    mtsExecutionResult result = Robot.BiasEncoder();
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "slot_qpbBiasEncAll: command failed \"" << result << "\"" << std::endl;
    }
}

void mtsRobotIO1394QtWidget::slot_qdsbWatchdogPeriod(double period_ms)
{
    mtsExecutionResult result = Robot.SetWatchdogPeriod(period_ms * cmn_ms);
    if(!result.IsOK()){
        CMN_LOG_CLASS_RUN_WARNING << "slot_qdsbWatchdogPeriod: command failed \""
                                  << result << "\"" << std::endl;
    }
}


void mtsRobotIO1394QtWidget::slot_qpbResetEnc()
{
    for (int i = 0; i < numOfAxis; i++) {
        if (sender() == qpbResetEnc[i]) {
            Actuators.ResetSingleEncoder(i);
        }
    }
}

void mtsRobotIO1394QtWidget::slot_qcbCurFBToggle(bool state)
{
    curFBState = state;
}

void mtsRobotIO1394QtWidget::slot_qdsbCurFBGain(double gain)
{
    curFBPGain = gain;
}

void mtsRobotIO1394QtWidget::slot_qdsbCurFBOffset(double offset)
{
    curFBOffset = offset;
}

void mtsRobotIO1394QtWidget::timerEvent(QTimerEvent *event)
{
    bool flag;
    Robot.IsValid(flag);
    if (flag) {
        Robot.GetPeriodStatistics(IntervalStatistics);
        Robot.GetPosition(jointPos); // vct
        jointPos.Multiply(cmn180_PI); // to degrees
        Actuators.GetPositionActuator(actuatorPosGet); // prm
        actuatorPos.Assign(actuatorPosGet.Position()); // vct
        actuatorPos.Multiply(cmn180_PI); // to degrees
        Robot.GetVelocity(vel);
        Robot.GetAnalogInputVolts(potVolt);
        Robot.GetAnalogInputPosSI(potPosSI);
        potPosSI.Multiply((cmn180_PI));
        Robot.GetMotorCurrent(motorFeedbackCurrent);
        Actuators.GetAmpEnable(ampEnable);
        Actuators.GetAmpStatus(ampStatus);
        Robot.GetPowerStatus(powerStatus);
        Robot.GetSafetyRelay(safetyRelay);
        Robot.GetWatchdogTimeout(watchdogTimeout);
    } else {
        jointPos.SetAll(tmpStatic);
        actuatorPos.SetAll(tmpStatic);
        vel.SetAll(tmpStatic);
        potVolt.SetAll(tmpStatic);
        potPosSI.SetAll(tmpStatic);
        motorFeedbackCurrent.SetAll(tmpStatic);
    }

    CMN_LOG_CLASS_RUN_VERBOSE << (osaGetTime() - startTime) << std::endl;

    tmpStatic += 0.1;

    JointPositionWidget->SetValue(jointPos);
    ActuatorPositionWidget->SetValue(actuatorPos);
    ActuatorVelocityWidget->SetValue(vel);
    PotVoltsWidget->SetValue(potVolt);
    PotPositionWidget->SetValue(potPosSI);
    CurrentFeedbackWidget->SetValue(motorFeedbackCurrent * 1000.0);

    updateRobotInfo();
}


////------------ Private Methods ----------------

void mtsRobotIO1394QtWidget::setupMenu()
{
    QMenu* fileMenu = this->menuBar()->addMenu("&File");

    fileMenu->addAction("E&xit", this, SLOT(close()),
                        QKeySequence(Qt::ALT + Qt::Key_F4));

}

void mtsRobotIO1394QtWidget::setupCisstInterface()
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
        robotInterface->AddFunction("GetMotorFeedbackCurrent", Robot.GetMotorCurrent);
        robotInterface->AddFunction("GetPowerStatus", Robot.GetPowerStatus);
        robotInterface->AddFunction("GetSafetyRelay", Robot.GetSafetyRelay);
        robotInterface->AddFunction("GetWatchdogTimeout", Robot.GetWatchdogTimeout);

        robotInterface->AddFunction("SetMotorCurrent", Robot.SetMotorCurrent);
        robotInterface->AddFunction("SetEncoderPosition", Robot.SetEncoderPosition);
        robotInterface->AddFunction("SetWatchdogPeriod", Robot.SetWatchdogPeriod);

        robotInterface->AddFunction("BiasCurrent", Robot.BiasCurrent);
        robotInterface->AddFunction("BiasEncoder", Robot.BiasEncoder);
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

        actuatorInterface->AddFunction("AnalogInVoltsToPosSI", Actuators.AnalogInVoltsToPosSI);
        actuatorInterface->AddFunction("DriveAmpsToBits", Actuators.DriveAmpsToBits);
    }

#if HAS_GC
    mtsInterfaceRequired * req;
    req = AddInterfaceRequired("GC");
    if (req) {
        req->AddFunction("Enable", GC.Enable);
        req->AddFunction("AdjustEncoders", GC.AdjustEncoders);
    }
#endif


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
    qcbEnableBoards = new QCheckBox("Enable boards");
    powerLayout->addWidget(qcbEnableBoards);
    qcbEnableAll = new QCheckBox("Enable all");
    powerLayout->addWidget(qcbEnableAll);
    powerLayout->addStretch(1);
    ampStatusButton = new QPushButton("Amp status: ON");
    powerLayout->addWidget(ampStatusButton);
    powerStatusButton = new QPushButton("Power status ON");
    powerLayout->addWidget(powerStatusButton);
    powerFrame->setLayout(powerLayout);
    powerFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // watchdog commands
    QVBoxLayout * watchdogLayout = new QVBoxLayout;
    QFrame * watchdogFrame = new QFrame;
    QLabel * watchdogTitle = new QLabel("watchdog");
    watchdogTitle->setFont(font);
    watchdogTitle->setAlignment(Qt::AlignCenter);
    watchdogLayout->addWidget(watchdogTitle);
    QHBoxLayout * watchdogSetLayout = new QHBoxLayout;
    {
        QLabel * wdogLabel = new QLabel("Wdog period (ms)");
        qdsbWatchdogPeriod = new QDoubleSpinBox;
        qdsbWatchdogPeriod->setMaximum(340.0); // max wdog_period = 340 ms
        qdsbWatchdogPeriod->setMinimum(0.0);
        qdsbWatchdogPeriod->setSingleStep(0.05);
        qdsbWatchdogPeriod->setValue(340.0); // default = 340 ms
        watchdogSetLayout->addWidget(wdogLabel);
        watchdogSetLayout->addWidget(qdsbWatchdogPeriod);
    }
    watchdogLayout->addLayout(watchdogSetLayout);
    watchdogLayout->addStretch(1);
    safetyRelayButton = new QPushButton("Safety relay: ON");
    watchdogLayout->addWidget(safetyRelayButton);
    watchdogButton = new QPushButton("Watchdog: OFF");
    watchdogLayout->addWidget(watchdogButton);
    watchdogFrame->setLayout(watchdogLayout);
    watchdogFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Encoder commands
    QVBoxLayout * encoderLayout = new QVBoxLayout;
    QFrame * encoderFrame = new QFrame;
    QLabel * encoderTitle = new QLabel("Encoders");
    encoderTitle->setFont(font);
    encoderTitle->setAlignment(Qt::AlignCenter);
    encoderLayout->addWidget(encoderTitle);
    qpbResetEncAll = new QPushButton("Reset all");
    encoderLayout->addWidget(qpbResetEncAll);
    qpbBiasEncAll = new QPushButton("Bias from potentiometers");
    encoderLayout->addWidget(qpbBiasEncAll);
    encoderLayout->addStretch(1);
    encoderFrame->setLayout(encoderLayout);
    encoderFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Current comands
    QVBoxLayout * currentLayout = new QVBoxLayout;
    QFrame * currentFrame = new QFrame;
    QLabel * currentTitle = new QLabel("Current");
    currentTitle->setFont(font);
    currentTitle->setAlignment(Qt::AlignCenter);
    currentLayout->addWidget(currentTitle);
    qcbEnableDirectControl = new QCheckBox("Direct control");
    qcbEnableDirectControl->setChecked(true);
    currentLayout->addWidget(qcbEnableDirectControl);
    qpbResetCurrentAll = new QPushButton("Reset all");
    currentLayout->addWidget(qpbResetCurrentAll);
    qpbBiasCurrentAll = new QPushButton("Bias from feedback");
    currentLayout->addWidget(qpbBiasCurrentAll);
    currentLayout->addStretch(1);
    currentFrame->setLayout(currentLayout);
    currentFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Commands layout
    QHBoxLayout * commandLayout = new QHBoxLayout;
    commandLayout->addWidget(powerFrame);
    commandLayout->addWidget(watchdogFrame);
    commandLayout->addWidget(encoderFrame);
    commandLayout->addWidget(currentFrame);


    // Feedbacks Label
    QGridLayout * gridLayout = new QGridLayout;
    int row = 0;

    vctBoolVec defaultEnable(numOfAxis, false);
    vctDoubleVec defaultCurrent(numOfAxis, 0.0);

    QLabel * axisEnableLabel = new QLabel("Axis power");
    gridLayout->addWidget(axisEnableLabel, row, 0);
    CurrentEnableEachWidget = new vctQtWidgetDynamicVectorBoolWrite();
    CurrentEnableEachWidget->SetValue(defaultEnable);
    gridLayout->addWidget(CurrentEnableEachWidget, row, 1);
    row++;

    QLabel * currentDesiredLabel = new QLabel("Desired current (mA)");
    gridLayout->addWidget(currentDesiredLabel, row, 0);
    CurrentSpinBoxWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    CurrentSpinBoxWidget->SetValue(defaultCurrent);
    gridLayout->addWidget(CurrentSpinBoxWidget, row, 1);
    row++;

    CurrentSliderWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SLIDER_WIDGET);
    CurrentSliderWidget->SetValue(defaultCurrent);
    gridLayout->addWidget(CurrentSliderWidget, row, 1);
    row++;

    QLabel * jointPosLabel = new QLabel("Joint positions (deg)");
    gridLayout->addWidget(jointPosLabel, row, 0);
    JointPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(JointPositionWidget, row, 1);
    row++;

    QLabel * actuatorPosLabel = new QLabel("Actuator positions (deg)");
    gridLayout->addWidget(actuatorPosLabel, row, 0);
    ActuatorPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(ActuatorPositionWidget, row, 1);
    row++;

    QLabel * velDegLabel = new QLabel("Velocities (deg/s)");
    gridLayout->addWidget(velDegLabel, row, 0);
    ActuatorVelocityWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(ActuatorVelocityWidget, row, 1);
    row++;

    QLabel * potVoltLabel = new QLabel("Analog inputs (V)");
    gridLayout->addWidget(potVoltLabel, row, 0);
    PotVoltsWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(PotVoltsWidget, row, 1);
    row++;

    QLabel * potPosSILabel = new QLabel("Potentiometers (deg)");
    gridLayout->addWidget(potPosSILabel, row, 0);
    PotPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(PotPositionWidget, row, 1);
    row++;

    QLabel * curmALabel = new QLabel("Current feedback (mA)");
    gridLayout->addWidget(curmALabel, row, 0);
    CurrentFeedbackWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(CurrentFeedbackWidget, row, 1);
    row++;


#if HAS_GC
    //----------------- GC Controller -----------
    QCheckBox* qcbGCEnable = new QCheckBox("Enable GC");
    QPushButton* qpbAdjustEncoder = new QPushButton("Adjust Encoder");
    quitButton = new QPushButton("Close");

    QHBoxLayout* gcLayout = new QHBoxLayout;
    gcLayout->addWidget(qcbGCEnable);
    gcLayout->addWidget(qpbAdjustEncoder);
    gcLayout->addStretch();
    gcLayout->addWidget(quitButton);
    QGroupBox* gcGroupBox = new QGroupBox("GC Controller");
    gcGroupBox->setLayout(gcLayout);

    connect(qcbGCEnable, SIGNAL(clicked(bool)), this, SLOT(slot_qcbGCEnable(bool)));
    connect(qpbAdjustEncoder, SIGNAL(clicked()), this, SLOT(slot_qpbAdjustEncoder()));
#endif


#if HAS_DEBUG_INFO
    // Debug Title
    // spacer          spacer
    // -----  Debug Info ------
    QGridLayout* debugTitleLayout = new QGridLayout;
    QSpacerItem* debugTitleLeftSpacer = new QSpacerItem(200, 20, QSizePolicy::Expanding);
    QSpacerItem* debugTitleRightSpacer = new QSpacerItem(200, 20, QSizePolicy::Expanding);
    debugTitleLayout->addItem(debugTitleLeftSpacer, 0, 0);
    debugTitleLayout->addItem(debugTitleRightSpacer, 0, 2);

    QFrame* debugTitleLeftLine = new QFrame;
    debugTitleLeftLine->setFrameShape(QFrame::HLine);
    debugTitleLeftLine->setFrameShadow(QFrame::Sunken);
    QFrame* debugTitleRightLine = new QFrame;
    debugTitleRightLine->setFrameShape(QFrame::HLine);
    debugTitleRightLine->setFrameShadow(QFrame::Sunken);
    QLabel* debugTitleLabel = new QLabel("Robot Info");
    // debugTitleLabel->setFont(font);
    debugTitleLabel->setAlignment(Qt::AlignCenter);

    debugTitleLayout->addWidget(debugTitleLeftLine, 1, 0);
    debugTitleLayout->addWidget(debugTitleLabel, 1, 1);
    debugTitleLayout->addWidget(debugTitleRightLine, 1, 2);

    // debug lower left
    debugTextEdit = new QTextEdit;
    debugTextEdit->setStyleSheet("color: blue;"
                                 "background-color: yellow;"
                                 "selection-color: yellow;"
                                 "selection-background-color: blue;");
    debugTextEdit->setText("Hello world!");

    QHBoxLayout* debugLowerLayout = new QHBoxLayout;
    debugLowerLayout->addWidget(debugTextEdit);
    debugLowerLayout->addStretch();

    QVBoxLayout* debugLayout = new QVBoxLayout;
    debugLayout->addLayout(debugTitleLayout);
    debugLayout->addLayout(debugLowerLayout);
#endif

    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addLayout(commandLayout);
    mainLayout->addLayout(gridLayout);
#if HAS_DEBUG_INFO
    mainLayout->addLayout(debugLayout);
#endif

#if HAS_GC
    mainLayout->addWidget(gcGroupBox);
#endif

    QFrame * mainFrame = new QFrame;
    mainFrame->setLayout(mainLayout);
    setCentralWidget(mainFrame);

    setWindowTitle(QString(this->GetName().c_str()));
    resize(sizeHint());

    // connect signals & slots
    connect(qcbEnableBoards, SIGNAL(toggled(bool)), this, SLOT(slot_qcbEnableBoards(bool)));
    connect(qcbEnableAll, SIGNAL(toggled(bool)), this, SLOT(slot_qcbEnableAll(bool)));
    connect(qcbEnableDirectControl, SIGNAL(toggled(bool)), this, SLOT(slot_qcbEnableDirectControl(bool)));
    connect(qpbResetCurrentAll, SIGNAL(clicked()), this, SLOT(slot_qpbResetCurrentAll()));
    connect(qpbBiasCurrentAll, SIGNAL(clicked()), this, SLOT(slot_qpbBiasCurrentAll()));

    connect(qpbResetEncAll, SIGNAL(clicked()), this, SLOT(slot_qpbResetEncAll()));
    connect(qpbBiasEncAll, SIGNAL(clicked()), this, SLOT(slot_qpbBiasEncAll()));
    connect(qdsbWatchdogPeriod, SIGNAL(valueChanged(double)),
            this, SLOT(slot_qdsbWatchdogPeriod(double)));
    connect(CurrentEnableEachWidget, SIGNAL(valueChanged()), this, SLOT(slot_qcbEnable()));
    connect(CurrentSpinBoxWidget, SIGNAL(valueChanged()), this, SLOT(slot_qdsbMotorCurrent_valueChanged()));
    connect(CurrentSliderWidget, SIGNAL(valueChanged()), this, SLOT(slot_qsliderMotorCurrent_valueChanged()));

    // set initial value
    qcbEnableBoards->setChecked(false);
    qcbEnableAll->setChecked(false);
}


void mtsRobotIO1394QtWidget::updateRobotInfo(void)
{
    // debug info
    if(!debugStream.str().empty()){
        std::cout << debugStream.str() << std::endl;
        debugTextEdit->clear();
        debugTextEdit->setText(debugStream.str().c_str());
        // clear
        debugStream.clear();
        debugStream.str("");
    }


    // amplifier status
    bool ampStatusGood = true;
    for (unsigned int i = 0; i < ampStatus.size(); i++) {
        ampStatusGood &= ampStatus[i];
    }
    if (ampStatusGood) {
        ampStatusButton->setText("Amp Status: ON");
        ampStatusButton->setStyleSheet("QPushButton { background-color: green }");
    } else {
        ampStatusButton->setText("Amp Status: OFF");
        ampStatusButton->setStyleSheet("QPushButton { background-color: red }");
    }

    // power status
    if (powerStatus) {
        powerStatusButton->setText("Power Enable: ON");
        powerStatusButton->setStyleSheet("QPushButton { background-color: green }");
    } else {
        powerStatusButton->setText("Power Enable: OFF");
        powerStatusButton->setStyleSheet("QPushButton { background-color: red }");
    }

    // safety Relay
    if (safetyRelay) {
        safetyRelayButton->setText("Safety Relay: ON");
        safetyRelayButton->setStyleSheet("QPushButton { background-color: green }");
    } else {
        safetyRelayButton->setText("Safety Relay: OFF");
        safetyRelayButton->setStyleSheet("QPushButton { background-color: red }");
    }

    // watchdog timeout
    if (watchdogTimeout) {
        watchdogButton->setText("Watchdog Timeout: TRUE");
        watchdogButton->setStyleSheet("QPushButton { background-color: red }");
    } else {
        watchdogButton->setText("Watchdog Timeout: FALSE");
        watchdogButton->setStyleSheet("QPushButton { background-color: green }");
    }
}
