/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen
  Created on: 2012-07-20

  (C) Copyright 2012-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsRobotIO1394QtWidget_h
#define _mtsRobotIO1394QtWidget_h

#include <cisstCommonXML.h>
#include <cisstOSAbstraction/osaTimeServer.h>
#include <cisstVector/vctQtWidgetDynamicVector.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsIntervalStatistics.h>
#include <cisstParameterTypes/prmPositionJointGet.h>

#include <QtCore>
#include <QtGui>

#define HAS_GC  0
#define HAS_DEBUG_INFO 1


class mtsRobotIO1394QtWidget: public QMainWindow, public mtsComponent
{
    Q_OBJECT
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsRobotIO1394QtWidget(const std::string & taskName, unsigned int numberOfActuators);
    // For now, single arg constructor hard-codes number of actuators. Could replace this
    // by a custom constructor arg.
    mtsRobotIO1394QtWidget(const std::string & taskName);
    inline ~mtsRobotIO1394QtWidget(void) {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

    inline std::stringstream & GetOutputStream(void) { return debugStream; }

protected:
    void Init(void);
    virtual void closeEvent(QCloseEvent *event);

private slots:
    void slot_qcbEnableBoards(bool toggle);
    void slot_qcbEnableAll(bool toggle);
    void slot_qcbEnable(void);
    void slot_qpbResetCurrentAll(void);
    void slot_qpbBiasCurrentAll(void);
    void slot_qdsbMotorCurrent_valueChanged();
    void slot_qsliderMotorCurrent_valueChanged(void);
    void slot_qpbResetEncAll(void);
    void slot_qpbBiasEncAll(void);
    void slot_qdsbWatchdogPeriod(double period_ms);
    void slot_qpbResetEnc(void);
    void slot_qcbCurFBToggle(bool);
    void slot_qdsbCurFBGain(double);
    void slot_qdsbCurFBOffset(double);

#if HAS_GC
    void slot_qcbGCEnable(bool toggle){
        GC.Enable(mtsBool(toggle));
        std::cout << "Enable: " << toggle << std::endl;
    }

    void slot_qpbAdjustEncoder(void){
        GC.AdjustEncoders();
    }
#endif


    void timerEvent(QTimerEvent * event);

private:
    void setupMenu(void);
    void setupCisstInterface(void);
    void setupUi(void);

    // gui update
    void updateCurrentDisplay(void);
    void updateRobotInfo(void);

    void currentPID(void);

    // unit conversion calculating
    double DACtoCurrents(long count);
    long CurrentstoDAC(double amp);


protected:
    std::stringstream debugStream;
    enum { DEBUG_START_LINE = 15 };
    unsigned int last_debug_line;

    struct RobotStruct {
        mtsFunctionRead GetPeriodStatistics;
        mtsFunctionRead GetNumberOfActuators;
        mtsFunctionRead IsValid;
        mtsFunctionVoid EnablePower;
        mtsFunctionVoid DisablePower;
        mtsFunctionVoid EnableSafetyRelay;
        mtsFunctionVoid DisableSafetyRelay;

        mtsFunctionRead GetPosition;
        mtsFunctionRead GetVelocity;
        mtsFunctionRead GetAnalogInputVolts;
        mtsFunctionRead GetAnalogInputPosSI;
        mtsFunctionRead GetMotorCurrent;
        mtsFunctionRead GetPowerStatus;
        mtsFunctionRead GetSafetyRelay;
        mtsFunctionRead GetWatchdogTimeout;

        mtsFunctionWrite SetMotorCurrent;
        mtsFunctionWrite SetEncoderPosition;
        mtsFunctionWrite SetWatchdogPeriod;

        mtsFunctionWrite BiasCurrent;
        mtsFunctionVoid BiasEncoder;
    } Robot;

    struct ActuatorStruct {
        mtsFunctionWrite SetAmpEnable;
        mtsFunctionWrite ResetSingleEncoder;

        mtsFunctionRead GetAmpEnable;
        mtsFunctionRead GetAmpStatus;
        mtsFunctionRead GetPositionActuator;

        mtsFunctionQualifiedRead AnalogInVoltsToPosSI;
        mtsFunctionQualifiedRead DriveAmpsToBits;
    } Actuators;

#if HAS_GC
    struct GCStruct {
        mtsFunctionWrite Enable;
        mtsFunctionVoid AdjustEncoders;
    } GC;
#endif


private:
    mtsIntervalStatistics IntervalStatistics;

    int numOfAxis;
    bool curFBState;
    double curFBPGain;
    double curFBOffset;
    vctLongVec encCnt;
    vctLongVec velCnt;
    vctLongVec potCnt;
    vctLongVec curCnt;

    vctDoubleVec jointPos;
    prmPositionJointGet actuatorPosGet;
    vctDoubleVec actuatorPos;
    vctDoubleVec vel;
    vctDoubleVec potVolt;
    vctDoubleVec potPosSI;
    vctDoubleVec motorFeedbackCurrent;
    vctDoubleVec motorControlCurrent;
    vctBoolVec ampEnable;
    vctBoolVec ampStatus;
    bool powerStatus;
    unsigned short safetyRelay;
    bool watchdogTimeout;

    // Interface
    double tmpStatic;
    vctDynamicVector<bool> lastEnableState;
    mtsInterfaceRequired * reqQLA;
    mtsInterfaceRequired * reqQLARaw;
    double startTime;

    // GUI: Commands
    QCheckBox * qcbEnableBoards;
    QFrame * cmdLowerInfoFrame;
    QCheckBox * qcbEnableAll;
    QCheckBox ** qcbEnable;
    QPushButton * qpbResetCurrentAll;
    QPushButton * qpbBiasCurrentAll;
    vctQtWidgetDynamicVectorBoolWrite * CurrentEnableEachWidget;
    vctQtWidgetDynamicVectorDoubleWrite * CurrentSpinBoxWidget;
    vctQtWidgetDynamicVectorDoubleWrite * CurrentSliderWidget;

    // GUI: Feedbacks
    QPushButton* qpbResetEncAll;
    QPushButton* qpbBiasEncAll;
    QDoubleSpinBox* qdsbWatchdogPeriod;
    QCheckBox* qcbEnableDirectControl;

    vctQtWidgetDynamicVectorDoubleRead * JointPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead * ActuatorPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead * ActuatorVelocityWidget;
    vctQtWidgetDynamicVectorDoubleRead * PotVoltsWidget;
    vctQtWidgetDynamicVectorDoubleRead * PotPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead * CurrentFeedbackWidget;

    QPushButton** qpbResetEnc;

    // GUI: Debug
    QTextEdit* debugTextEdit;
    QPushButton* ampStatusButton;
    QPushButton* powerStatusButton;
    QPushButton* safetyRelayButton;
    QPushButton* watchdogButton;

    // Control
    QPushButton* quitButton;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobotIO1394QtWidget);

#endif // _mtsRobotIO1394QtWidget_h
