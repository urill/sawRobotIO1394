
sawRobotIO1394
==============

This SAW package contains code for interfacing with the QLA (Quad Linear
Amplifier) library.

Partitioning
------------

This package will be partitioned into the current CISST MultiTask component
(mtsRobotIO1394) as well as some CISST-independent source code (osaRobotIO1394).

### Contents

#### code

##### `DigitalInInternal.[h|cpp]` class representing a digitial input.
- *Pre-partitioning*
    - Defines a CISST provided interface for a digital input as a button
    - Generates button events whwnever GetData is called and the value changes
    - Accesses digital input with `AmpIO::GetDigitalInput()`
- *Post-partitioning*
    - Unchanged

##### `RobotInternal.[h|cpp]` class representing a "robot" (collection of joints)

- *Pre-partitioning*
    - Defines structures describing calibration
        - `ActuatorInfo` Declares nested structs which contain onversions
          for motors, encoders, analog inputs between bits, amps, and SI
          units. These structures map directly to the XML hierarchy.
    - `Configure()` parses the XML file to get bit/current/SI conversions
        - `ConfigureCoupling()` parses the XML file further to get the
          transmision matrix.
        - `UpdateInternalConfiguration()` resizes joint structures and computes
          maximum motor current feedback **120%** and adds 50mA? Shouldn''t these
          be parameterized somewhere?
        - Zeros out torque commands
    - `CheckIfValid()` verifies each amplifier (board) is not null and
    - `GetData()` queries the amplifiers (boards) for mechanism state and
      stores it in member variables.
    - `ConvertRawToSI()` converts bits to SI units for all inputs and
      outputs and stores them in member variables.
    - `UpdateStateAndEvents()` 
        - Checks if motor current feedback is too high, disables power if it
          is
        - Stores commanded and measured currents and re-biases if necessary
          by calling `ResetAmpsToBitsOffsetUsingFeedbackAmps()` which
          computes the average difference between the commanded and measured
          currents and then applies that to the command offset. '''NOTE''': This
          function computes the current bias update with the following formula:
              ```
              bias = mean(amps_commanded) - mean(amps_measured)
              ```
          This will work for large samples sizes, but for small sample sizes,
          depending on when the current is commanded and sampled, the first
          value could include the measurement from the previous command and the
          last value will not be captured.
    - `[Enable|Disable]Power()` enables power for all amplifiers and enables
      them
    - `[Enable|Disable]SafetyRelay()` ???
    - `SetWatchdogPeriod()` 
    - `SetTorqueJoint()` 
        - Stores commanded torque in a member variable
        - Computes actuator torque, then actuator current,
        - Calls `SetMotorCurrent()`
            - Stores commanded current in a member variable
            - Converts actuator current to bits
            - Calls `SetMotorCurrentRAW()`
                - Stores current command bits in a member variable
                - Sets the motor current for each amplifier
    - `RequestAmpsToBitsOffsetUsingFeedbackAmps()` sets the size of the bias
      measurement function
    - `RequestAmpsToBitsOffsetUsingFeedbackAmps()` computes the bew current
      command bias (see `UpdateStateAndEvents()` above)
    - `ResetEncoderOffsetUsingPotPosSI()` biases the encoders based on the
      current commanded position and the measured position so that commanded
      = actual
        - stores the bias in the `Actuator` structures
    - `ResetSingleEncoder()` sets a given encoder to 0
    - `SetEncoderPosition()` 
        - Stores the declared encoder position in a member variable
        - Calls `EncoderSIToRaw()` to convert the angle to bits
            - Calls `SetEncoderPositionRaw()` to set the encoder position
                - Sets the encoder positions for each amplifier
    - Conversions
        - `EncoderRawToSI()` converts encoder position bits to angle
        - `EncoderSIToRaw()` converts encoder angle to bits
        - `EncoderRawToDeltaPosSI()` converts encoder velocity bits to angular
          velocity
        - `EncoderRawToDeltaPosT()` change in time (ms)
        - `DriveAmpsToBits()` converts motor currents to bits
        - `DriveBitsToFeedbackAmps()` converts bits to measured motor currents
        - `DriveNmToAmps()` converts torques to currents
        - `DriveAmpsToNm()` converts currents to torques
        - `AnalogInBitsToVolts()` converts bits to measured input volts on a pot
        - `AnalogInVoltsToPosSI()` converts measured input volts to position on
          a pot
- *Post-partitioning*
    - Get rid of `ActuatorInfo` structures, and just accumulate vectors of
      parameters.
    - Separate XML parsing with structure manipulation
    - Parameterize maximum motor current feedback

##### `mtsRobotIO1394.cpp` CISST MultiTask Component
- *Pre-partitioning*
    - Definition of the `mtsRobotIO1394` CISST component member functions
    - Constructor constructs `FirewirePort` and checks number of users
    - `mtsRobotIO1394::Configure` accepts a filename to configure the robot
      from. This uses `cmnXMLPath` internally.
        - Constructs `DigitalInInternal` objects.
        - Constructs `AmpIO` objects for each board in `BoardList`.
        - Constructs `RobotInternal` objects for each robot.
            - Configures each robot
    - `Run()` is the main loop for using the boards
    
        ```cpp
        void mtsRobotIO1394::Run(void)
        {
            size_t i;
            // Read from all boards
            Port->ReadAllBoards();
            // Loop through the robots, processing feedback
            for (i = 0; i < RobotList.size(); i++) {
                if (RobotList[i]->CheckIfValid()) {
                    // Copy data to state table
                    RobotList[i]->GetData();
                    // Convert from raw to SI units (TBD)
                    RobotList[i]->ConvertRawToSI();
                    //
                    RobotList[i]->UpdateStateAndEvents();
                }
            }
            // Loop through the digital inputs
            for (i = 0; i < DigitalInList.size(); i++) {
                DigitalInList[i]->GetData();
            }
            // Invoke connected components (if any)
            RunEvent();
            // Process queued commands (e.g., to set motor current)
            ProcessQueuedCommands();
            // Write to all boards
            Port->WriteAllBoards();
        }
        ```


- *Post-partitioning*
    - Above the line

#### examples
#### include
#### share
#### tests

### Doppelgangers

#### CISST Vector

Create Eigen3 classes which implement necessary CISST Vector interfaces.

#### Streams to Define

These are CISST ostreams that would need to be defined in a CISST-independent context.

- `CMN_LOG_CLASS_INIT_ERROR`
- `CMN_LOG_CLASS_INIT_VERBOSE`
- `CMN_LOG_CLASS_RUN_ERROR`
