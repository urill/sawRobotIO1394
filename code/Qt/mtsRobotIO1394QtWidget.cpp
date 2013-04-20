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
    if(ret == QMessageBox::Yes){
        Robot.DisablePower();
        event->accept();
    }else {
        event->ignore();
    }
}


//----------- Private Slot ------------------------------

void mtsRobotIO1394QtWidget::slot_qcbEnableAll(bool toggle)
{
#if 1
    if(toggle){
        Robot.EnablePower();
    }else{
        Robot.DisablePower();
    }
#endif

//    update GUI
    for (int i = 0; i < numOfAxis; i++){
        qcbEnable[i]->blockSignals(true);
        qcbEnable[i]->setChecked(toggle);
        qcbEnable[i]->blockSignals(false);
    }
}

void mtsRobotIO1394QtWidget::slot_qpbResetCurrentAll(void)
{
    // set GUI value
    vctDoubleVec cmdCurA(numOfAxis);
    cmdCurA.SetAll(0.0);
    Robot.SetMotorCurrent(cmdCurA);
    for (int i = 0; i < numOfAxis; i++){
        qsliderMotorCurrent[i]->blockSignals(true);
        qsliderMotorCurrent[i]->setValue(32768);
        qdsbMotorCurrent[i]->setValue(0.0);
        qsliderMotorCurrent[i]->blockSignals(false);
    }
}

void mtsRobotIO1394QtWidget::slot_qpbBiasCurrentAll(void)
{
    mtsExecutionResult result = Robot.BiasCurrent(mtsInt(1000)); // use a 1000 samples to average current feedback
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "slot_qpbBiasCurrentAll: command failed \"" << result << "\"" << std::endl;
    }
}

void mtsRobotIO1394QtWidget::slot_qcbEnable(bool CMN_UNUSED(toggle))
{
    for(int i = 0; i < numOfAxis; i++){
        ampEnable[i] = qcbEnable[i]->isChecked();
    }
    Actuators.SetAmpEnable(ampEnable);
}


void mtsRobotIO1394QtWidget::slot_qdsbMotorCurrent_valueChanged()
{
    vctDoubleVec cmdCurmA;
    cmdCurmA.SetSize(numOfAxis);
    vctLongVec cmdCurCnt;
    cmdCurCnt.SetSize(numOfAxis);

    // get value from GUI
    for(int i = 0; i < numOfAxis; i++)
        cmdCurmA[i] = qdsbMotorCurrent[i]->value();

#if SWITCH
    vctDoubleVec cmdCurA(numOfAxis);
    cmdCurA = cmdCurmA.Divide(1000.0);
    Robot.SetMotorCurrent(cmdCurA);
    Actuators.DriveAmpsToBits(cmdCurA, cmdCurCnt);
//    Robot.MotorCurrentToDAC(cmdCurmA, cmdCurCnt);
#else
    for (int i = 0; i < numOfAxis; i++){
        cmdCurCnt[i] = CurrentstoDAC(cmdCurmA[i]);
    }
#endif

    // set GUI value
    for (int i = 0; i < numOfAxis; i++){
        qsliderMotorCurrent[i]->blockSignals(true);
        qsliderMotorCurrent[i]->setValue(cmdCurCnt[i]);
        qsliderMotorCurrent[i]->blockSignals(false);
    }
}


void mtsRobotIO1394QtWidget::slot_qsliderMotorCurrent_valueChanged()
{
    vctDoubleVec cmdCurmA;
    cmdCurmA.SetSize(numOfAxis);
    vctLongVec cmdCurCnt;
    cmdCurCnt.SetSize(numOfAxis);

    vctDoubleVec cmdCurA(numOfAxis);

    // get value from GUI
    for(int i = 0; i < numOfAxis; i++)
        cmdCurCnt[i] = qsliderMotorCurrent[i]->value();

#if SWITCH
    for (int i = 0; i < numOfAxis; i++){
        cmdCurmA[i] = DACtoCurrents(cmdCurCnt[i]);
    }

    cmdCurA = cmdCurmA.Divide(1000.0);
    Robot.SetMotorCurrent(cmdCurA);
#else
    for (int i = 0; i < numOfAxis; i++){
        cmdCurmA[i] = DACtoCurrents(cmdCurCnt[i]);
    }
#endif

    // set GUI value
    for (int i = 0; i < numOfAxis; i++){
        qdsbMotorCurrent[i]->blockSignals(true);
        qdsbMotorCurrent[i]->setValue(cmdCurmA[i]);
        qdsbMotorCurrent[i]->blockSignals(false);
    }
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

    // std::cerr << IntervalStatistics << std::endl;
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

        actuatorInterface->AddFunction("SetAmpEnable", Actuators.SetAmpEnable);
        actuatorInterface->AddFunction("ResetSingleEncoder", Actuators.ResetSingleEncoder);

        actuatorInterface->AddFunction("GetAmpEnable", Actuators.GetAmpEnable);
        actuatorInterface->AddFunction("GetAmpStatus", Actuators.GetAmpStatus);

        actuatorInterface->AddFunction("AnalogInVoltsToPosSI", Actuators.AnalogInVoltsToPosSI);
        actuatorInterface->AddFunction("DriveAmpsToBits", Actuators.DriveAmpsToBits);
    }

    mtsInterfaceRequired * req;
#if HAS_GC
    req = AddInterfaceRequired("GC");
    if (req) {
        req->AddFunction("Enable", GC.Enable);
        req->AddFunction("AdjustEncoders", GC.AdjustEncoders);
    }
#endif


}



void mtsRobotIO1394QtWidget::setupUi()
{
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    //----------------- Command -------------------------------
    // Commands Title
    // spacer          spacer
    // -----  Commands ------
    QGridLayout* cmdTitleLayout = new QGridLayout;
    QSpacerItem* cmdTitleLeftSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    QSpacerItem* cmdTitleRightSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    cmdTitleLayout->addItem(cmdTitleLeftSpacer, 0, 0);
    cmdTitleLayout->addItem(cmdTitleRightSpacer, 0, 2);

    QFrame* cmdTitleLeftLine = new QFrame;
    cmdTitleLeftLine->setFrameShape(QFrame::HLine);
    cmdTitleLeftLine->setFrameShadow(QFrame::Sunken);
    QFrame* cmdTitleRightLine = new QFrame;
    cmdTitleRightLine->setFrameShape(QFrame::HLine);
    cmdTitleRightLine->setFrameShadow(QFrame::Sunken);
    QLabel* cmdTitleLabel = new QLabel("Commands");
    cmdTitleLabel->setFont(font);
    cmdTitleLabel->setAlignment(Qt::AlignCenter);

    cmdTitleLayout->addWidget(cmdTitleLeftLine, 1, 0);
    cmdTitleLayout->addWidget(cmdTitleLabel, 1, 1);
    cmdTitleLayout->addWidget(cmdTitleRightLine, 1, 2);

    // Commands Label
    // [] Enable All
    // Motor Currents
    //    QLabel
    QVBoxLayout* cmdLabelLayout = new QVBoxLayout;
    QFrame* cmdLabelFrame = new QFrame;
    qcbEnableAll = new QCheckBox("Enable All");
    QLabel* motorCurLabel = new QLabel("Motor Current");
//    motorCurLabel->setAlignment(Qt::AlignRight);
    cmdLabelLayout->addWidget(qcbEnableAll);
    qpbResetCurrentAll = new QPushButton("Reset All");
    cmdLabelLayout->addWidget(qpbResetCurrentAll);
    qpbBiasCurrentAll = new QPushButton("Bias All");
    cmdLabelLayout->addWidget(qpbBiasCurrentAll);
    cmdLabelLayout->addWidget(motorCurLabel);
    cmdLabelLayout->addWidget(new QLabel(""));
    cmdLabelFrame->setLayout(cmdLabelLayout);
    cmdLabelFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Commands Info
    // [] Enable Axis i
    //   xx.xx mA  double spin box
    //   --|--     slider
    QVBoxLayout** cmdInfoLayout = new QVBoxLayout*[numOfAxis];
    QFrame** cmdInfoFrame = new QFrame*[numOfAxis];
    qcbEnable = new QCheckBox*[numOfAxis];
    qdsbMotorCurrent = new QDoubleSpinBox*[numOfAxis];
    qsliderMotorCurrent = new QSlider*[numOfAxis];

    for(int i = 0; i < numOfAxis; i++){
        qcbEnable[i] = new QCheckBox("Enable " + QString::number(i+1));
        qdsbMotorCurrent[i] = new QDoubleSpinBox;
        qdsbMotorCurrent[i]->setSuffix(" mA");
        qdsbMotorCurrent[i]->setDecimals(2);
        qdsbMotorCurrent[i]->setSingleStep(3.0);
        qdsbMotorCurrent[i]->setMinimum(-6250);
        qdsbMotorCurrent[i]->setMaximum(6250);
        qdsbMotorCurrent[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        qsliderMotorCurrent[i] = new QSlider;
        qsliderMotorCurrent[i]->setOrientation(Qt::Horizontal);
        qsliderMotorCurrent[i]->setMinimum(0);
        qsliderMotorCurrent[i]->setMaximum(65535);
        qsliderMotorCurrent[i]->setSingleStep(100);
        qsliderMotorCurrent[i]->setPageStep(500);

        cmdInfoLayout[i] = new QVBoxLayout;
        cmdInfoLayout[i]->addWidget(qcbEnable[i]);
        cmdInfoLayout[i]->addWidget(qdsbMotorCurrent[i]);
        cmdInfoLayout[i]->addWidget(qsliderMotorCurrent[i]);

        cmdInfoFrame[i] = new QFrame;
        cmdInfoFrame[i]->setLayout(cmdInfoLayout[i]);
        cmdInfoFrame[i]->setFrameShape(QFrame::StyledPanel);
        cmdInfoFrame[i]->setFrameShadow(QFrame::Sunken);
    }

    // Commands lower layout
    // cmdLabel | cmdInfo1 | cmdInfo2 |...
    QHBoxLayout* cmdLowerLayout = new QHBoxLayout;
    cmdLowerLayout->addWidget(cmdLabelFrame);
    for(int i = 0; i < numOfAxis; i++){
        cmdLowerLayout->addWidget(cmdInfoFrame[i]);
    }

    // Commands layout
    // cmdTitleLayout
    // cmdLowerLayout
    QVBoxLayout* cmdLayout = new QVBoxLayout;
    cmdLayout->addLayout(cmdTitleLayout);
    cmdLayout->addLayout(cmdLowerLayout);


    //---------------------- Feedback ----------------------------

    // Commands Title
    // spacer          spacer
    // -----  Feedback ------
    QGridLayout* fbTitleLayout = new QGridLayout;
    QSpacerItem* fbTitleLeftSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    QSpacerItem* fbTitleRightSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    fbTitleLayout->addItem(fbTitleLeftSpacer, 0, 0);
    fbTitleLayout->addItem(fbTitleRightSpacer, 0, 2);

    QFrame* fbTitleLeftLine = new QFrame;
    fbTitleLeftLine->setFrameShape(QFrame::HLine);
    fbTitleLeftLine->setFrameShadow(QFrame::Sunken);
    QFrame* fbTitleRightLine = new QFrame;
    fbTitleRightLine->setFrameShape(QFrame::HLine);
    fbTitleRightLine->setFrameShadow(QFrame::Sunken);
    QLabel* fbTitleLabel = new QLabel("Feedbacks");
    fbTitleLabel->setFont(font);
    fbTitleLabel->setAlignment(Qt::AlignCenter);

    fbTitleLayout->addWidget(fbTitleLeftLine, 1, 0);
    fbTitleLayout->addWidget(fbTitleLabel, 1, 1);
    fbTitleLayout->addWidget(fbTitleRightLine, 1, 2);

    // Feedbacks Label
    QGridLayout* fbLayout = new QGridLayout;
    // QFrame* fbFrame = new QFrame;
    QLabel* jointPosLabel = new QLabel("Pos. joints (deg)");
    QLabel* actuatorPosLabel = new QLabel("Pos. actuators (deg)");
    QLabel* velDegLabel = new QLabel("Velocity (deg/s)");
    QLabel* potVoltLabel = new QLabel("PotMeter (V)");
    QLabel* potPosSILabel = new QLabel("PotMeter (deg)");
    QLabel* curmALabel = new QLabel("Current (mA)");
    qpbResetEncAll = new QPushButton("Reset Enc");
    qpbBiasEncAll = new QPushButton("Bias Enc/Pot");
    QLabel* wdogLabel = new QLabel("Wdog Period (ms)");
    qdsbWatchdogPeriod = new QDoubleSpinBox;
    qdsbWatchdogPeriod->setMaximum(340.0);
    qdsbWatchdogPeriod->setMinimum(0.0);
    qdsbWatchdogPeriod->setSingleStep(0.05);
    qdsbWatchdogPeriod->setValue(0);
    QHBoxLayout* fbLowerLayout = new QHBoxLayout;
    fbLowerLayout->addWidget(qpbResetEncAll);
    fbLowerLayout->addWidget(qpbBiasEncAll);
    fbLowerLayout->addStretch();
    fbLowerLayout->addWidget(wdogLabel);
    fbLowerLayout->addWidget(qdsbWatchdogPeriod);
    fbLowerLayout->addStretch();


    fbLayout->addWidget(jointPosLabel, 0, 0);
    fbLayout->addWidget(actuatorPosLabel, 1, 0);
    fbLayout->addWidget(velDegLabel, 2, 0);
    fbLayout->addWidget(potVoltLabel, 3, 0);
    fbLayout->addWidget(potPosSILabel, 4, 0);
    fbLayout->addWidget(curmALabel, 5, 0);
//    fbLayout->addWidget(qpbResetEncAll, 6, 0);
//    fbLayout->addWidget(qpbBiasEncAll, 7, 0);
    fbLayout->addLayout(fbLowerLayout, 6, 0, 1, 2);

    JointPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    fbLayout->addWidget(JointPositionWidget->GetWidget(), 0, 1);
    ActuatorPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    fbLayout->addWidget(ActuatorPositionWidget->GetWidget(), 1, 1);
    ActuatorVelocityWidget = new vctQtWidgetDynamicVectorDoubleRead();
    fbLayout->addWidget(ActuatorVelocityWidget->GetWidget(), 2, 1);
    PotVoltsWidget = new vctQtWidgetDynamicVectorDoubleRead();
    fbLayout->addWidget(PotVoltsWidget->GetWidget(), 3, 1);
    PotPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    fbLayout->addWidget(PotPositionWidget->GetWidget(), 4, 1);
    CurrentFeedbackWidget = new vctQtWidgetDynamicVectorDoubleRead();
    fbLayout->addWidget(CurrentFeedbackWidget->GetWidget(), 5, 1);

    // fbFrame->setLayout(fbLayout);
    // fbFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

#if 0
    //----------------- Control ------------------------
    QCheckBox* qcbCurFBToggle = new QCheckBox("CurPID");
    QDoubleSpinBox* qdsbCurFBGain = new QDoubleSpinBox;
    qdsbCurFBGain->setMinimum(-3.0);
    qdsbCurFBGain->setMaximum(3.0);
    qdsbCurFBGain->setSingleStep(0.05);
    qdsbCurFBGain->setValue(curFBPGain);
    QDoubleSpinBox* qdsbCurFBOffset = new QDoubleSpinBox;
    qdsbCurFBOffset->setMinimum(-100.0);
    qdsbCurFBOffset->setMaximum(100.0);
    qdsbCurFBOffset->setSingleStep(0.5);
    qdsbCurFBOffset->setSuffix("mA");
    qdsbCurFBOffset->setValue(curFBOffset);

    quitButton = new QPushButton("Close");

    QHBoxLayout* ctrlLayout = new QHBoxLayout;
    ctrlLayout->addWidget(qcbCurFBToggle);
    ctrlLayout->addWidget(new QLabel("PGain"));
    ctrlLayout->addWidget(qdsbCurFBGain);
    ctrlLayout->addWidget(new QLabel("Offset"));
    ctrlLayout->addWidget(qdsbCurFBOffset);
    ctrlLayout->addStretch();
    ctrlLayout->addWidget(quitButton);
    QGroupBox* ctrlGroupBox = new QGroupBox("Control");
    ctrlGroupBox->setLayout(ctrlLayout);
#endif

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
    QSpacerItem* debugTitleLeftSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    QSpacerItem* debugTitleRightSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    debugTitleLayout->addItem(debugTitleLeftSpacer, 0, 0);
    debugTitleLayout->addItem(debugTitleRightSpacer, 0, 2);

    QFrame* debugTitleLeftLine = new QFrame;
    debugTitleLeftLine->setFrameShape(QFrame::HLine);
    debugTitleLeftLine->setFrameShadow(QFrame::Sunken);
    QFrame* debugTitleRightLine = new QFrame;
    debugTitleRightLine->setFrameShape(QFrame::HLine);
    debugTitleRightLine->setFrameShadow(QFrame::Sunken);
    QLabel* debugTitleLabel = new QLabel("Robot Info");
    debugTitleLabel->setFont(font);
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

    // debug lower right
    // ZC: TODO REDO this
    ampEnableButton = new QPushButton("Amp Enable: ON");
    ampStatusButton = new QPushButton("Amp Status: ON");
    powerStatusButton = new QPushButton("PowerStatus ON");
    safetyRelayButton = new QPushButton("SafetyRelay: ON");
    watchdogButton = new QPushButton("Watchdog: OFF");

    QVBoxLayout* debugLowerRightLeyout = new QVBoxLayout;
    debugLowerRightLeyout->addWidget(ampEnableButton);
    debugLowerRightLeyout->addWidget(ampStatusButton);
    debugLowerRightLeyout->addWidget(powerStatusButton);
    debugLowerRightLeyout->addWidget(safetyRelayButton);
    debugLowerRightLeyout->addWidget(watchdogButton);

    QHBoxLayout* debugLowerLayout = new QHBoxLayout;
    debugLowerLayout->addWidget(debugTextEdit);
    debugLowerLayout->addLayout(debugLowerRightLeyout);
    debugLowerLayout->addStretch();

    QVBoxLayout* debugLayout = new QVBoxLayout;
    debugLayout->addLayout(debugTitleLayout);
    debugLayout->addLayout(debugLowerLayout);
#endif


    // main layout
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(cmdLayout);
    mainLayout->addLayout(fbLayout);

#if HAS_DEBUG_INFO
    mainLayout->addLayout(debugLayout);
#endif

#if HAS_GC
    mainLayout->addWidget(gcGroupBox);
#endif



    QFrame* mainFrame = new QFrame;
    mainFrame->setLayout(mainLayout);
    setCentralWidget(mainFrame);
//    setFixedWidth(750);
//    setFixedHeight(sizeHint().height());

    setWindowTitle(QString(this->GetName().c_str()));
    resize(sizeHint());

    // connect signals & slots
    // Commands
    connect(qcbEnableAll, SIGNAL(toggled(bool)), this, SLOT(slot_qcbEnableAll(bool)));
    connect(qpbResetCurrentAll, SIGNAL(clicked()), this, SLOT(slot_qpbResetCurrentAll()));
    connect(qpbBiasCurrentAll, SIGNAL(clicked()), this, SLOT(slot_qpbBiasCurrentAll()));

    connect(qpbResetEncAll, SIGNAL(clicked()), this, SLOT(slot_qpbResetEncAll()));
    connect(qpbBiasEncAll, SIGNAL(clicked()), this, SLOT(slot_qpbBiasEncAll()));
    connect(qdsbWatchdogPeriod, SIGNAL(valueChanged(double)),
            this, SLOT(slot_qdsbWatchdogPeriod(double)));
    for(int i = 0; i < numOfAxis; i++){
        connect(qcbEnable[i], SIGNAL(toggled(bool)), this, SLOT(slot_qcbEnable(bool)));
        connect(qdsbMotorCurrent[i], SIGNAL(valueChanged(double)),
                this, SLOT(slot_qdsbMotorCurrent_valueChanged()));
        connect(qsliderMotorCurrent[i], SIGNAL(valueChanged(int)),
                this, SLOT(slot_qsliderMotorCurrent_valueChanged()));
        // adeguet1 connect(qpbResetEnc[i], SIGNAL(clicked()), this, SLOT(slot_qpbResetEnc()));
    }

    // Control
//    connect(qcbCurFBToggle, SIGNAL(clicked(bool)), this, SLOT(slot_qcbCurFBToggle(bool)));
//    connect(qdsbCurFBGain, SIGNAL(valueChanged(double)), this, SLOT(slot_qdsbCurFBGain(double)));

    connect(quitButton, SIGNAL(clicked()), this, SLOT(close()));


    // set initial value
    qcbEnableAll->setChecked(false);
    for(int i = 0; i < numOfAxis; i++){
        qsliderMotorCurrent[i]->blockSignals(true);
        qsliderMotorCurrent[i]->setValue(MOTORCUR_DAC/2);
        qsliderMotorCurrent[i]->blockSignals(false);
    }
    slot_qsliderMotorCurrent_valueChanged();
}


void mtsRobotIO1394QtWidget::updateRobotInfo()
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

    // status

//    if(ampEnable.at(0)){
//        ampEnableButton->setText("Amp Enable: ON");
//        ampEnableButton->setStyleSheet("QPushButton { background-color: green }");
//    }else{
//        ampEnableButton->setText("Amp Enable: OFF");
//        ampEnableButton->setStyleSheet("QPushButton { background-color: red }");
//    }

    // power status
    if(powerStatus){
        powerStatusButton->setText("Power Enable: ON");
        powerStatusButton->setStyleSheet("QPushButton { background-color: green }");
    }else{
        powerStatusButton->setText("Power Enable: OFF");
        powerStatusButton->setStyleSheet("QPushButton { background-color: red }");
    }

    // safety Relay
    if(safetyRelay){
        safetyRelayButton->setText("Safety Relay: ON");
        safetyRelayButton->setStyleSheet("QPushButton { background-color: green }");
    }else{
        safetyRelayButton->setText("Safety Relay: OFF");
        safetyRelayButton->setStyleSheet("QPushButton { background-color: red }");
    }

    // safety Relay
    if(watchdogTimeout){
        watchdogButton->setText("Watchdog Timeout: TRUE");
        watchdogButton->setStyleSheet("QPushButton { background-color: red }");
    }else{
        watchdogButton->setText("Watchdog Timeout: FALSE");
        watchdogButton->setStyleSheet("QPushButton { background-color: green }");
    }
}


double mtsRobotIO1394QtWidget::DACtoCurrents(long count){
    return ((double)count / MOTORCUR_DAC * MOTORCUR_MAX * 2 - MOTORCUR_MAX);
}


long mtsRobotIO1394QtWidget::CurrentstoDAC(double amp){
    long val;
    val = (long) (amp + MOTORCUR_MAX)/(MOTORCUR_MAX * 2.0) * double(MOTORCUR_DAC);
    std::cout << val << std::endl;
    return val;
}

