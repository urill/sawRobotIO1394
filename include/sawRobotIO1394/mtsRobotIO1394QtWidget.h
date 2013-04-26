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
// #define HAS_DEBUG_INFO 1
#define HAS_DIGITAL 0


/*!
  \todo use cisst naming convention
  \todo figure out what to do with the debug window, this is not per robot, most likely per port - cisstLog
  \todo maybe rename this class to mtsRobotIO1394{Robot,DigitalInputs,Log}QtWidget and create using mtsRobotIO1394FactoryQtWidget
  \todo cisst Qt convention is now to start with the Qt prefix, i.e. mtsQtWidgetRobotIO1394 ...
  \todo add commands to retrieve maximum current allowed per axis and apply to current widgets
  \todo use events to monitor amp status
  */
class mtsRobotIO1394QtWidget: public QMainWindow, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsRobotIO1394QtWidget(const std::string & componentName, unsigned int numberOfActuators);
    mtsRobotIO1394QtWidget(const mtsComponentConstructorNameAndUInt &arg);
    inline ~mtsRobotIO1394QtWidget(void) {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    void Init(void);
    virtual void closeEvent(QCloseEvent *event);

private slots:
    void slot_qcbEnableAmps(bool toggle);
    void slot_qcbEnableAll(bool toggle);
    void slot_qcbEnableDirectControl(bool toggle);
    void slot_qcbEnable(void);
    void slot_qpbResetCurrentAll(void);
    void slot_qpbBiasCurrentAll(void);
    void slot_qdsbMotorCurrent_valueChanged();
    void slot_qsliderMotorCurrent_valueChanged(void);
    void slot_qpbResetEncAll(void);
    void slot_qpbBiasEncAll(void);
    void slot_qdsbWatchdogPeriod(double period_ms);
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
    void SetupCisstInterface(void);
    void setupUi(void);

    // gui update
    void UpdateCurrentDisplay(void);
    void UpdateRobotInfo(void);

protected:
    bool DirectControl;
    int TimerPeriodInMilliseconds;
    double WatchdogPeriodInSeconds;

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
        mtsFunctionRead GetMotorCurrentMax;
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
        mtsFunctionVoid EnableBoardsPower;
        mtsFunctionVoid DisableBoardsPower;

        mtsFunctionWrite SetAmpEnable;
        mtsFunctionWrite ResetSingleEncoder;

        mtsFunctionRead GetAmpEnable;
        mtsFunctionRead GetAmpStatus;
        mtsFunctionRead GetPositionActuator;
    } Actuators;

#if HAS_GC
    struct GCStruct {
        mtsFunctionWrite Enable;
        mtsFunctionVoid AdjustEncoders;
    } GC;
#endif


private:
    mtsIntervalStatistics IntervalStatistics;

    int NumberOfActuators;
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
    QCheckBox * EnableAmps;
    QFrame * cmdLowerInfoFrame;
    QCheckBox * qcbEnableAll;
    QPushButton * qpbResetCurrentAll;
    QPushButton * qpbBiasCurrentAll;

    // GUI: Feedbacks
    QPushButton* qpbResetEncAll;
    QPushButton* qpbBiasEncAll;
    QDoubleSpinBox* qdsbWatchdogPeriod;
    QCheckBox* qcbEnableDirectControl;

    vctQtWidgetDynamicVectorBoolWrite * CurrentEnableEachWidget;
    vctQtWidgetDynamicVectorBoolRead * AmpStatusEachWidget;
    vctQtWidgetDynamicVectorDoubleWrite * CurrentSpinBoxWidget;
    vctQtWidgetDynamicVectorDoubleWrite * CurrentSliderWidget;
    vctQtWidgetDynamicVectorDoubleRead * JointPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead * ActuatorPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead * ActuatorVelocityWidget;
    vctQtWidgetDynamicVectorDoubleRead * PotVoltsWidget;
    vctQtWidgetDynamicVectorDoubleRead * PotPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead * CurrentFeedbackWidget;

    QPushButton* ampStatusButton;
    QPushButton* powerStatusButton;
    QPushButton* safetyRelayButton;
    QPushButton* watchdogButton;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobotIO1394QtWidget);

#endif // _mtsRobotIO1394QtWidget_h
