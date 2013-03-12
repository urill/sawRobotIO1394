/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsNDISerialToolQtComponent.h 3536 2012-03-13 02:25:59Z adeguet1 $

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
#include <cisstMultiTask/mtsComponent.h>

#include <QtCore>
#include <QtGui>

#define HAS_GC  1
#define HAS_PID 0
#define HAS_DEBUG_INFO 1


class mtsRobotIO1394QtWidget: public QMainWindow, public mtsComponent
{
    Q_OBJECT
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsRobotIO1394QtWidget(const std::string & taskName);
    inline ~mtsRobotIO1394QtWidget(void) {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

    inline std::stringstream & GetOutputStream(void) { return debugStream; }

protected:
    virtual void closeEvent(QCloseEvent *event);

private slots:
    void slot_qcbEnableAll(bool toggle);
    void slot_qcbEnable(bool toggle);
    void slot_qdsbMotorCurrent_valueChanged(void);
    void slot_qsliderMotorCurrent_valueChanged(void);
    void slot_qpbResetEncAll(void);
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

#if HAS_PID
    void slot_qcbPIDEnable(bool toggle){
        PID.Enable(mtsBool(toggle));
    }
#endif

    void timerEvent(QTimerEvent * event);

private:
    void setupMenu(void);
    void setupCisstInterface(void);
    void setupUi(void);

    // gui update
    void updateEncoderDisplay(void);
    void updateVelocityDisplay(void);
    void updatePotDisplay(void);
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
        mtsFunctionRead GetNumberOfActuators;
        mtsFunctionRead IsValid;
        mtsFunctionVoid EnablePower;
        mtsFunctionVoid DisablePower;
        mtsFunctionVoid EnableSafetyRelay;
        mtsFunctionVoid DisableSafetyRelay;
        mtsFunctionWrite SetAmpEnable;

        mtsFunctionRead GetPosition;
        mtsFunctionRead GetVelocity;
        mtsFunctionRead GetAnalogInput;
        mtsFunctionRead GetMotorCurrent;
        mtsFunctionRead GetAmpEnable;
        mtsFunctionRead GetAmpStatus;
        mtsFunctionRead GetPowerStatus;
        mtsFunctionRead GetSafetyRelay;

        mtsFunctionWrite SetMotorCurrent;
        mtsFunctionWrite SetEncoderPosition;

        mtsFunctionQualifiedRead PotVoltsToDegree;
        mtsFunctionQualifiedRead MotorCurrentToDAC;
    } Robot;

#if HAS_GC
    struct GCStruct {
        mtsFunctionWrite Enable;
        mtsFunctionVoid AdjustEncoders;
    } GC;
#endif

#if HAS_PID
    struct PIDStruct {
        mtsFunctionWrite Enable;
    } PID;
#endif


private:
    int numOfAxis;
    bool curFBState;
    double curFBPGain;
    double curFBOffset;
    vctLongVec encCnt;
    vctLongVec velCnt;
    vctLongVec potCnt;
    vctLongVec curCnt;

    vctDoubleVec pos;
    vctDoubleVec vel;
    vctDoubleVec analogIn;
    vctDoubleVec motorFeedbackCurrent;
    vctDoubleVec motorControlCurrent;
    vctBoolVec ampEnable;
    vctBoolVec ampStatus;
    bool powerStatus;
    unsigned short safetyRelay;

    // Interface
    double tmpStatic;
    vctDynamicVector<bool> lastEnableState;
    mtsInterfaceRequired * reqQLA;
    mtsInterfaceRequired * reqQLARaw;
    double startTime;

    // GUI: Commands
    QCheckBox* qcbEnableAll;
    QCheckBox** qcbEnable;
    QDoubleSpinBox** qdsbMotorCurrent;
    QSlider** qsliderMotorCurrent;

    // GUI: Feedbacks
    QPushButton* qpbResetEncAll;
    QLineEdit** qleEncDeg;
    QLineEdit** qleVelDeg;
    QLineEdit** qlePotVolt;
    QLineEdit** qleCurmA;
    QPushButton** qpbResetEnc;

    // GUI: Debug
    QTextEdit* debugTextEdit;
    QPushButton* ampEnableButton;
    QPushButton* ampStatusButton;
    QPushButton* powerStatusButton;
    QPushButton* safetyRelayButton;

    // Control
    QPushButton* quitButton;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobotIO1394QtWidget);

#endif // _mtsRobotIO1394QtWidget_h
