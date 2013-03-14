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

#include <cisstMultiTask/mtsInterfaceRequired.h>

#define SWITCH 0

// Find a better way for user to get these values
#define MOTORCUR_MAX 6500       // max motor current in mA
#define MOTORCUR_DAC 0xFFFF     // max dac value for motor current


CMN_IMPLEMENT_SERVICES_DERIVED(mtsRobotIO1394QtWidget, mtsComponent);


mtsRobotIO1394QtWidget::mtsRobotIO1394QtWidget(const std::string &taskName)
    :mtsComponent(taskName)
{
    numOfAxis = 4;
    curFBState = false;
    curFBPGain = 1.0;
    curFBOffset = 30.0;
    tmpStatic = 0;
    lastEnableState.SetSize(numOfAxis);
    lastEnableState.SetAll(false);

    pos.SetSize(numOfAxis);
    vel.SetSize(numOfAxis);
    analogIn.SetSize(numOfAxis);
    motorFeedbackCurrent.SetSize(numOfAxis);
    motorFeedbackCurrent.SetAll(0.0);
    ampEnable.SetSize(numOfAxis);

    startTime = QDateTime::currentMSecsSinceEpoch();

    setupMenu();
    setupCisstInterface();
    setupUi();

    startTimer(30); // ms
}



void mtsRobotIO1394QtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsRobotIO1394QtWidget::Startup()
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
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
#if SWITCH
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

void mtsRobotIO1394QtWidget::slot_qcbEnable(bool CMN_UNUSED(toggle))
{
    for(int i = 0; i < numOfAxis; i++){
        ampEnable[i] = qcbEnable[i]->isChecked();
    }
    Robot.SetAmpEnable(ampEnable);
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
    Robot.SetMotorCurrent(cmdCurmA);
    Robot.MotorCurrentToDAC(cmdCurmA, cmdCurCnt);
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

    // get value from GUI
    for(int i = 0; i < numOfAxis; i++)
        cmdCurCnt[i] = qsliderMotorCurrent[i]->value();

#if SWITCH
    for (int i = 0; i < numOfAxis; i++){
        cmdCurmA[i] = DACtoCurrents(cmdCurCnt[i]);
    }
    Robot.SetMotorCurrent(cmdCurmA);
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
#if SWITCH
//    for(int i = 0; i < numOfAxis; i++){
//        QLA.ResetEncoders(mtsShort(i));
//    }

    std::cout << "ResetAllEncAll" << std::endl;
#else
    std::cout << "ResetAllEncAll" << std::endl;
#endif
}

void mtsRobotIO1394QtWidget::slot_qpbResetEnc()
{
    for (int i = 0; i < numOfAxis; i++){
        if(sender() == qpbResetEnc[i]){
//            QLA.ResetEncoders(mtsShort(i));
            std::cout << "ResetAllEncAll: " << i << std::endl;
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
        Robot.GetPosition(pos);
        Robot.GetVelocity(vel);
        Robot.GetAnalogInput(analogIn);
        Robot.GetMotorCurrent(motorFeedbackCurrent);
        Robot.GetAmpEnable(ampEnable);
        Robot.GetAmpStatus(ampStatus);
        Robot.GetPowerStatus(powerStatus);
        Robot.GetSafetyRelay(safetyRelay);
    }else{
        pos.SetAll(tmpStatic);
        vel.SetAll(tmpStatic);
        analogIn.SetAll(tmpStatic);
        motorFeedbackCurrent.SetAll(tmpStatic);
    }

    CMN_LOG_CLASS_RUN_VERBOSE << (QDateTime::currentMSecsSinceEpoch() - startTime)/1000.0 << std::endl;

//    CMN_LOG_CLASS_RUN_ERROR << motorFeedbackCurrent << std::endl;


    tmpStatic += 0.1;
    updateEncoderDisplay();
    updateVelocityDisplay();
    updatePotDisplay();
    updateCurrentDisplay();
    updateRobotInfo();


//    if (!debugStream.str().empty()) {
//        int cur_line = DEBUG_START_LINE;
//        char line[80];
//        memset(line, ' ', sizeof(line));
//        for (i = cur_line; i < last_debug_line; i++)
//            mvprintw(i, 9, line);
//        while (!debugStream.eof()) {
//            debugStream.getline(line, sizeof(line));
//            mvprintw(cur_line++, 9, line);
//        }
//        debugStream.clear();
//        debugStream.str("");
//        last_debug_line = cur_line;
//    }
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
    mtsInterfaceRequired *req = AddInterfaceRequired("Robot");
    if (req) {
        req->AddFunction("GetNumberOfActuators", Robot.GetNumberOfActuators);
        req->AddFunction("IsValid", Robot.IsValid);
        req->AddFunction("EnablePower", Robot.EnablePower);
        req->AddFunction("DisablePower", Robot.DisablePower);
        req->AddFunction("EnableSafetyRelay", Robot.EnableSafetyRelay);
        req->AddFunction("DisableSafetyRelay", Robot.DisableSafetyRelay);
        req->AddFunction("SetAmpEnable", Robot.SetAmpEnable);

        req->AddFunction("GetPosition", Robot.GetPosition);
        req->AddFunction("GetVelocity", Robot.GetVelocity);
        // adeguet1: temporary - req->AddFunction("GetAnalogInput", Robot.GetAnalogInput);
        req->AddFunction("GetMotorFeedbackCurrent", Robot.GetMotorCurrent);
        req->AddFunction("GetAmpEnable", Robot.GetAmpEnable);
        req->AddFunction("GetAmpStatus", Robot.GetAmpStatus);
        req->AddFunction("GetPowerStatus", Robot.GetPowerStatus);
        req->AddFunction("GetSafetyRelay", Robot.GetSafetyRelay);

        req->AddFunction("SetMotorCurrent", Robot.SetMotorCurrent);
        // adeguet1: temporary - req->AddFunction("SetEncoderPosition", Robot.SetEncoderPosition);

        // adeguet1: temporary - req->AddFunction("PotVoltsToDegree", Robot.PotVoltsToDegree);
        // adeguet1: temporary - req->AddFunction("MotorCurrentToDAC", Robot.MotorCurrentToDAC);
    }

#if HAS_GC
    req = AddInterfaceRequired("GC");
    if (req) {
        req->AddFunction("Enable", GC.Enable);
        req->AddFunction("AdjustEncoders", GC.AdjustEncoders);
    }
#endif

#if HAS_PID
    req = AddInterfaceRequired("PID");
    if (req) {
        req->AddFunction("Enable", PID.Enable);
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
        qdsbMotorCurrent[i]->setMinimum(-6500);
        qdsbMotorCurrent[i]->setMaximum(6500);
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
    QVBoxLayout* fbLabelLayout = new QVBoxLayout;
    QFrame* fbLabelFrame = new QFrame;
    QLabel* encDegLabel = new QLabel("Encoder (deg)"); // motor deg
    QLabel* velDegLabel = new QLabel("Velocity (deg/s)");
    QLabel* potVolLabel = new QLabel("PotMeter (V)");
    QLabel* curmALabel = new QLabel("Current (mA)");
    qpbResetEncAll = new QPushButton("Reset Enc");
    fbLabelLayout->addWidget(encDegLabel);
    fbLabelLayout->addWidget(velDegLabel);
    fbLabelLayout->addWidget(potVolLabel);

    fbLabelLayout->addWidget(curmALabel);
    fbLabelLayout->addWidget(qpbResetEncAll);
    fbLabelFrame->setLayout(fbLabelLayout);
    fbLabelFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Feedbacks Info
    QVBoxLayout** fbInfoLayout = new QVBoxLayout*[numOfAxis];
    QFrame** fbInfoFrame = new QFrame*[numOfAxis];

    qleEncDeg = new QLineEdit*[numOfAxis];
    qleVelDeg = new QLineEdit*[numOfAxis];
    qlePotVolt = new QLineEdit*[numOfAxis];
    qleCurmA = new QLineEdit*[numOfAxis];
    qpbResetEnc = new QPushButton*[numOfAxis];

    for(int i = 0; i < numOfAxis; i++){
        qleEncDeg[i] = new QLineEdit;
        qleVelDeg[i] = new QLineEdit;
        qlePotVolt[i] = new QLineEdit;
        qleCurmA[i] = new QLineEdit;
        qpbResetEnc[i] = new QPushButton("Reset Enc " + QString::number(i+1));

        qleEncDeg[i]->setAlignment(Qt::AlignRight);
        qleVelDeg[i]->setAlignment(Qt::AlignRight);
        qlePotVolt[i]->setAlignment(Qt::AlignRight);
        qleCurmA[i]->setAlignment(Qt::AlignRight);

        fbInfoLayout[i] = new QVBoxLayout;
        fbInfoLayout[i]->addWidget(qleEncDeg[i]);
        fbInfoLayout[i]->addWidget(qleVelDeg[i]);
        fbInfoLayout[i]->addWidget(qlePotVolt[i]);
        fbInfoLayout[i]->addWidget(qleCurmA[i]);
        fbInfoLayout[i]->addWidget(qpbResetEnc[i]);

        fbInfoFrame[i] = new QFrame;
        fbInfoFrame[i]->setLayout(fbInfoLayout[i]);
        fbInfoFrame[i]->setFrameShape(QFrame::StyledPanel);
        fbInfoFrame[i]->setFrameShadow(QFrame::Sunken);
    }

    // Commands lower layout
    // fbLabel | fbInfo1 | fbInfo2 |...
    QHBoxLayout* fbLowerLayout = new QHBoxLayout;
    fbLowerLayout->addWidget(fbLabelFrame);
    for(int i = 0; i < numOfAxis; i++){
        fbLowerLayout->addWidget(fbInfoFrame[i]);
    }


    // Feedbacks layout
    // fbTitleLayout
    // fbLowerLayout
    QVBoxLayout* fbLayout = new QVBoxLayout;
    fbLayout->addLayout(fbTitleLayout);
    fbLayout->addLayout(fbLowerLayout);

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

    quitButton = new QPushButton("Quit");

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
    quitButton = new QPushButton("Quit");

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

#if HAS_PID
    QCheckBox* qcbPIDEnable = new QCheckBox("Enable PID");
    QHBoxLayout* pidLayout = new QHBoxLayout;
    pidLayout->addWidget(qcbPIDEnable);
    QGroupBox* pidGroupBox = new QGroupBox("PID Controller");
    pidGroupBox->setLayout(pidLayout);

    connect(qcbPIDEnable, SIGNAL(clicked(bool)), this, SLOT(slot_qcbPIDEnable(bool)));
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

    QVBoxLayout* debugLowerRightLeyout = new QVBoxLayout;
    debugLowerRightLeyout->addWidget(ampEnableButton);
    debugLowerRightLeyout->addWidget(ampStatusButton);
    debugLowerRightLeyout->addWidget(powerStatusButton);
    debugLowerRightLeyout->addWidget(safetyRelayButton);

    QHBoxLayout* debugLowerLayout = new QHBoxLayout;
    debugLowerLayout->addWidget(debugTextEdit);
    debugLowerLayout->addLayout(debugLowerRightLeyout);
    debugLowerLayout->addStretch();

//    QGridLayout* debugLayout = new QGridLayout;
//    debugLayout->addLayout(debugTitleLayout, 0, 0, 3, 1);
//    debugLayout->addWidget(debugTextEdit, 1, 0, 1, 1);
//    debugLayout->addLayout(debugLowerRightLeyout, 1, 2, 1, 1);
//    debugLayout->addWidget();

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

#if HAS_PID
    mainLayout->addWidget(pidGroupBox);
#endif


    QFrame* mainFrame = new QFrame;
    mainFrame->setLayout(mainLayout);
    setCentralWidget(mainFrame);
//    setFixedWidth(750);
//    setFixedHeight(sizeHint().height());

    setWindowTitle("LoPoMoCo 1394 QLA Board Test GUI");
    resize(sizeHint());

    // connect signals & slots
    // Commands
    connect(qcbEnableAll, SIGNAL(toggled(bool)), this, SLOT(slot_qcbEnableAll(bool)));
    connect(qpbResetEncAll, SIGNAL(clicked()), this, SLOT(slot_qpbResetEncAll()));
    for(int i = 0; i < numOfAxis; i++){
        connect(qcbEnable[i], SIGNAL(toggled(bool)), this, SLOT(slot_qcbEnable(bool)));
        connect(qdsbMotorCurrent[i], SIGNAL(valueChanged(double)),
                this, SLOT(slot_qdsbMotorCurrent_valueChanged()));
        connect(qsliderMotorCurrent[i], SIGNAL(valueChanged(int)),
                this, SLOT(slot_qsliderMotorCurrent_valueChanged()));
        connect(qpbResetEnc[i], SIGNAL(clicked()), this, SLOT(slot_qpbResetEnc()));
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


void mtsRobotIO1394QtWidget::updateEncoderDisplay()
{
    // update GUI
    std::stringstream ssDeg;
    ssDeg << std::fixed << std::setprecision(2);

    for (int i = 0; i < numOfAxis; i++){
        ssDeg.str("");
        ssDeg << pos.at(i);
        qleEncDeg[i]->setText(ssDeg.str().c_str());
    }

    CMN_LOG_CLASS_RUN_VERBOSE << pos << std::endl;
}

void mtsRobotIO1394QtWidget::updateVelocityDisplay()
{
    // update GUI
    std::stringstream ssDeg;
    ssDeg << std::fixed << std::setprecision(4);

    for (int i = 0; i < numOfAxis; i++){
        ssDeg.str("");
        ssDeg << vel.at(i);
        qleVelDeg[i]->setText(ssDeg.str().c_str());
    }

    CMN_LOG_CLASS_RUN_VERBOSE << vel << std::endl;
}


void mtsRobotIO1394QtWidget::updatePotDisplay()
{
    // update GUI
    std::stringstream ssVolt;
    ssVolt << std::fixed << std::setprecision(4);

    for (int i = 0; i < numOfAxis; i++){
        ssVolt.str("");
        ssVolt << analogIn.at(i);
        qlePotVolt[i]->setText(ssVolt.str().c_str());
    }
}

void mtsRobotIO1394QtWidget::updateCurrentDisplay()
{
    // update GUI
    std::stringstream ssmA;
    ssmA << std::fixed << std::setprecision(2);

    for (int i = 0; i < numOfAxis; i++){
        ssmA.str("");
        ssmA << motorFeedbackCurrent.at(i);
        qleCurmA[i]->setText(ssmA.str().c_str());
    }

    CMN_LOG_CLASS_RUN_VERBOSE << motorFeedbackCurrent << std::endl;
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

//    Robot.GetAmpEnable(ampEnable);
//    Robot.GetAmpStatus(ampStatus);
//    Robot.GetPowerStatus(powerStatus);
//    Robot.GetSafetyRelay(safetyRelay);

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

