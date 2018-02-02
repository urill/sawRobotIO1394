mEncoderPositionBits vctIntVec
mEncoderVelocityBits vctIntVec

mActuatorCurrentFeedback vctDoubleVec
mPotVoltage vctDoubleVec

robotInterface->AddEventWrite(EventTriggers.EncoderSetTo, "EncoderSetTo", mEncoderPosition);
actuatorInterface->AddCommandReadState(*mStateTableRead, mActuatorPowerEnabled,
                                           "GetAmpEnable"); // vector[bool]
    // Exposed IO
    robotInterface->AddCommandWrite(&mtsRobot1394::IOSetEncoderPositionBits, thisBase,
                                    "IOSetEncoderPositionBits");
    robotInterface->AddCommandWrite(&mtsRobot1394::IOSetEncoderVelocityBits, thisBase,
                                    "IOSetEncoderVelocityBits");
    robotInterface->AddCommandWrite(&mtsRobot1394::IOSetPotVoltage, thisBase,
                                    "IOSetPotVoltage");
    robotInterface->AddCommandWrite(&mtsRobot1394::IOSetActuatorCurrentFeedback, thisBase,
                                    "IOSetActuatorCurrentFeedback");