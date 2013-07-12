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

/*!
  \todo maybe rename this class to mtsRobotIO1394{Robot,DigitalInputs,Log}QtWidget and create using mtsRobotIO1394FactoryQtWidget
  \todo cisst Qt convention is now to start with the Qt prefix, i.e. mtsQtWidgetRobotIO1394 ...
  */
class mtsRobotIO1394QtWidget: public QWidget, public mtsComponent
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
    void SlotEnableAmps(bool toggle);
    void SlotEnableAll(bool toggle);
    void SlotEnableDirectControl(bool toggle);
    void SlotEnable(void);
    void SlotResetCurrentAll(void);
    void SlotBiasCurrentAll(void);
    void SlotMotorCurrentValueChanged();
    void SlotSliderMotorCurrentValueChanged(void);
    void SlotResetEncodersAll(void);
    void SlotBiasEncodersAll(void);
    void SlotWatchdogPeriod(double period_ms);

    void timerEvent(QTimerEvent * event);

private:
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
        mtsFunctionRead GetMotorRequestedCurrent;
        mtsFunctionRead GetMotorFeedbackCurrent;
        mtsFunctionRead GetMotorCurrentMax;
        mtsFunctionRead GetJointType;
        mtsFunctionRead GetPowerStatus;
        mtsFunctionRead GetSafetyRelay;
        mtsFunctionRead GetWatchdogTimeout;
        mtsFunctionRead GetAmpTemperature;

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

private:
    mtsIntervalStatistics IntervalStatistics;

    size_t NumberOfActuators;

    vctDoubleVec UnitFactor;
    vctDoubleVec JointPosition;
    prmPositionJointGet ActuatorPositionGet;
    vctDoubleVec ActuatorPosition;
    vctDoubleVec ActuatorVelocity;
    vctDoubleVec PotentiometersVolts;
    vctDoubleVec PotentiometersPosition;
    vctDoubleVec MotorFeedbackCurrent;
    vctDoubleVec MotorControlCurrent;
    vctBoolVec AmpEnable;
    vctBoolVec AmpStatus;
    bool PowerStatus;
    unsigned short SafetyRelay;
    bool WatchdogTimeout;
    vctDoubleVec AmpTemperature;

    // Interface
    double DummyValueWhenNotConnected;
    vctDynamicVector<bool> LastEnableState;
    double StartTime;

    // GUI: Commands
    QCheckBox * QCBEnableAmps;
    QCheckBox * QCBEnableAll;
    QPushButton * QPBResetCurrentAll;
    QPushButton * QPBBiasCurrentAll;

    // GUI: Feedbacks
    QPushButton * QPBResetEncAll;
    QPushButton * QPBBiasEncAll;
    QDoubleSpinBox * QSBWatchdogPeriod;
    QCheckBox * QCBEnableDirectControl;

    vctQtWidgetDynamicVectorBoolWrite * QVWCurrentEnableEachWidget;
    vctQtWidgetDynamicVectorBoolRead * QVRAmpStatusEachWidget;
    vctQtWidgetDynamicVectorDoubleWrite * QVWCurrentSpinBoxWidget;
    vctQtWidgetDynamicVectorDoubleWrite * QVWCurrentSliderWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRJointPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRActuatorPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRActuatorVelocityWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRPotVoltsWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRPotPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRCurrentFeedbackWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRAmpTemperature;

    QPushButton * QPBAmpStatusButton;
    QPushButton * QPBPowerStatusButton;
    QPushButton * QPBSafetyRelayButton;
    QPushButton * QPBWatchdogButton;

    void PowerStatusEventHandler(const bool & status);

    // signal and slot used by PowerStatusEventHandler
signals:
    void SignalPowerStatus(bool status);
protected slots:
    void SlotPowerStatus(bool status);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobotIO1394QtWidget);

#endif // _mtsRobotIO1394QtWidget_h
