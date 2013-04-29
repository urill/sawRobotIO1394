function [ output_args ] = configGenerator( aCalName, aRobotName, aBoardID, aDigital)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here


% maybe I will turn this into a function

% TODOs
%  1. verify all settings
%  2. fix joint 8 (gripper)

% create temporary .m file
copyfile(aCalName, 'temp.m');

% define some constants for mXXX.cal file
UPPER_LIMIT = 1; LOWER_LIMIT = 2;
MST_JNT_POS_GR_DOFS = 8;
MST_MOT_DOFS = 8;
MST_JNT_POS_DOFS = 7;
temp;

numOfActuator = 8;
numOfJoint = 8;

% boardID1 = 5; boardID2 = 6;
boardID = aBoardID;


% master

% motor constants Unit: V
motorVol = [24 24 24 24 24 24 24 24];

% motor default current Unit: A
motorDefCur(1:MST_MOT_DOFS) = [0.67 0.67 0.67 0.67 0.59 0.59 0.407 0.0];

% motor max current Unit: A
motorMaxCur(1:MST_MOT_DOFS) = [0.67 0.67 0.67 0.92 0.75 0.59 0.407 0.0];

% motor toruqe const  Unit: Nm/A
motorTor = [0.0438 0.0438 0.0438 0.0438 0.00495 0.00495 0.00339 0.0];

% Gear ratio
gearRatio = [63.41 49.88 59.73 10.53 33.16 33.16 16.58 0.0];

% Encoder counts per turn
encCPT = [4000 4000 4000 4000 64 64 64 0];

% Pitch
% 1 for revolute, mm/deg for prismatic
pitch = [1 1 1 1 1 1 1 1];

% ==== POT =======
% raw value from Intuitive Surgical Inc mXXXX.cal file
% NOTE:
%  Intuitive system
%    1. 12 bit ADC
%    2. 0-5 V (Typical)
%    3. Unit: Radian
%
%  JHU QLA board
%    1. 16 bit ADC
%    2. 0-4.5 V
%    3. Unit: Radian
jointUpper = joint.signal_range(UPPER_LIMIT,:);
jointLower = joint.signal_range(LOWER_LIMIT,:);
potGain = motor.pot_input_gain;
potOffset = motor.pot_input_offset;


%%
% =============================================
% Compute XML values
% =============================================
% === Drive =======
% Direction
driveDirection = [-1 1 1 1 -1 1 -1 1];
AmpsToBitsScale = driveDirection .* 5242.8800;
AmpsToBitsOffset = ones(1, numOfActuator) .* (2^15);

BitsToFbAmpsScale = driveDirection .* 0.000190738;
BitsToFbAmpsOffset = driveDirection .* (-6.25);

NmToAmps = ones(1,numOfActuator) ./ gearRatio ./ motorTor;
MaxCurrent = motorDefCur;


% === Encoder ======
BitsToPosSIScale = driveDirection .* 360 ./ encCPT .* pitch ./ gearRatio;

% AmpIO buff = buff + MIDRANGE_VEL
% Velocity = deltaPos / deltaTime
% deltaPos = 360 / (encCPT/4) / gearRatio * pitch  || not quadratic
% deltaTime = timeCounter * 1 / 768000 (Unit: sec)
% Fast clock 49.152 MHz / 2^6 = 768 kHz
% Velocity = 360 * 768000 / (encCPT / 4.0) / gearRatio * pitch / timeCounter
%          = BitsToDeltaPosSIScale / timeCounter

BitsToDeltaPosSI = driveDirection .* 360.0 .* 768000 ./ (encCPT ./ 4.0) ./ gearRatio .* pitch;
BitsToDeltaT = ones(1,numOfActuator) * -1;
CountsPerTurn = encCPT;

% === AnalogIn =====
BitsToVolts = ones(1,numOfActuator) * 0.0000686656;
VoltsToPosSIScale = potGain * 2^12 / (4.5 - 0.0) * 180.0 / pi;
VoltsToPosSIOffset = potOffset * 180.0 / pi;



%% Create XML file

% ==============================
% Generate XML file
% Reference:
% Simple XML Node Creation
% http://blogs.mathworks.com/community/2010/09/13/simple-xml-node-creation/
% ==============================

fileName = 'MTML.xml';

docNode = com.mathworks.xml.XMLUtils.createDocument('Config');
Config = docNode.getDocumentElement;

% ------------- Robot ----------------
Robot = docNode.createElement('Robot');
Robot.setAttribute('Name', aRobotName);
Robot.setAttribute('NumOfActuator', num2str(numOfActuator));
Robot.setAttribute('NumOfJoint', num2str(numOfJoint));
Config.appendChild(Robot);

% Acutator array
for i = 1:numOfActuator
    Actuator = docNode.createElement('Actuator');
    Actuator.setAttribute('ActuatorID', num2str(i-1));
    % set to boardID1 & boardID2
    Actuator.setAttribute('BoardID', num2str(boardID( idivide(i-1, int32(4))+1 )));
    Actuator.setAttribute('AxisID', num2str(mod(i-1,4)));
    Actuator.setAttribute('Pos1', 'ENC');
    Actuator.setAttribute('Pos2', 'POT');
    Robot.appendChild(Actuator);
    
    % Drive
    Drive = docNode.createElement('Drive');
    Actuator.appendChild(Drive);
    
    X_Amps2Bits = docNode.createElement('AmpsToBits');
    X_Amps2Bits.setAttribute('Scale', num2str(AmpsToBitsScale(i), '%5.2f'));
    X_Amps2Bits.setAttribute('Offset', num2str(AmpsToBitsOffset(i), '%5.0f'));
    Drive.appendChild(X_Amps2Bits);
    X_BitsToFeedbackAmps = docNode.createElement('BitsToFeedbackAmps');
    X_BitsToFeedbackAmps.setAttribute('Scale', num2str(BitsToFbAmpsScale(i), '%5.9f'));
    X_BitsToFeedbackAmps.setAttribute('Offset', num2str(BitsToFbAmpsOffset(i), '%5.2f'));
    Drive.appendChild(X_BitsToFeedbackAmps);
    X_NmToAmps = docNode.createElement('NmToAmps');
    X_NmToAmps.setAttribute('Scale', num2str(NmToAmps(i), '%5.6f'));
    Drive.appendChild(X_NmToAmps);
    X_MaxCurrent = docNode.createElement('MaxCurrent');
    X_MaxCurrent.setAttribute('Value', num2str(MaxCurrent(i), '%5.3f'));
    X_MaxCurrent.setAttribute('Unit', 'A');
    Drive.appendChild(X_MaxCurrent);
    
    % Encoder
    Enc = docNode.createElement('Encoder');
    Actuator.appendChild(Enc);
    
    X_BitsToPosSI = docNode.createElement('BitsToPosSI');
    X_BitsToPosSI.setAttribute('Scale', num2str(BitsToPosSIScale(i), '%5.6f'));
    X_BitsToPosSI.setAttribute('Offset', '0');
    Enc.appendChild(X_BitsToPosSI);
    X_BitsToDeltaPosSI = docNode.createElement('BitsToDeltaPosSI');
    X_BitsToDeltaPosSI.setAttribute('Scale', num2str(BitsToDeltaPosSI(i), '%5.2f'));
    X_BitsToDeltaPosSI.setAttribute('Offset', '0');
    Enc.appendChild(X_BitsToDeltaPosSI);
    X_BitsToDeltaT = docNode.createElement('BitsToDeltaT');
    X_BitsToDeltaT.setAttribute('Scale', num2str(BitsToDeltaT(i), '%5.3f'));
    X_BitsToDeltaT.setAttribute('Offset', '0');
    Enc.appendChild(X_BitsToDeltaT);
    X_CountsPerTurn = docNode.createElement('CountsPerTurn');
    X_CountsPerTurn.setAttribute('Value', num2str(CountsPerTurn(i), '%5.0f'));
    Enc.appendChild(X_CountsPerTurn);
    
    % AnalogIn
    AnaglogIn = docNode.createElement('AnalogIn');
    Actuator.appendChild(AnaglogIn);
    
    X_BitsToVolts = docNode.createElement('BitsToVolts');
    % BitsToVolts
    X_BitsToVolts.setAttribute('Scale', num2str(BitsToVolts(i), 6));
    X_BitsToVolts.setAttribute('Offset', '0');
    AnaglogIn.appendChild(X_BitsToVolts);
    X_VoltsToPosSI = docNode.createElement('VoltsToPosSI');
    X_VoltsToPosSI.setAttribute('Scale', num2str(VoltsToPosSIScale(i), '%5.4f'));
    X_VoltsToPosSI.setAttribute('Offset', num2str(VoltsToPosSIOffset(i), '%5.4f'));
    AnaglogIn.appendChild(X_VoltsToPosSI);
end

% Potentiometers
Potentiometers = docNode.createElement('Potentiometers');
Potentiometers.setAttribute('Position', 'Joints');
Robot.appendChild(Potentiometers);

% Coupling
X_Coupling = docNode.createElement('Coupling');
X_Coupling.setAttribute('Value', num2str(1));
Robot.appendChild(X_Coupling);

% Coupling/ActuatorToJointPosition
X_ActuatorToJointPosition = docNode.createElement('ActuatorToJointPosition');
X_Coupling.appendChild(X_ActuatorToJointPosition);

ActuatorToJointPosition = cell(8,1);
ActuatorToJointPosition(1) = cellstr('1.00  0.00   0.00 0.00 0.00 0.00 0.00 0.00');
ActuatorToJointPosition(2) = cellstr('0.00  1.00   0.00 0.00 0.00 0.00 0.00 0.00');
ActuatorToJointPosition(3) = cellstr('0.00 -1.00   1.00 0.00 0.00 0.00 0.00 0.00');
ActuatorToJointPosition(4) = cellstr('0.00  0.00 0.6697 1.00 0.00 0.00 0.00 0.00');
ActuatorToJointPosition(5) = cellstr('0.00  0.00   0.00 0.00 1.00 0.00 0.00 0.00');
ActuatorToJointPosition(6) = cellstr('0.00  0.00   0.00 0.00 0.00 1.00 0.00 0.00');
ActuatorToJointPosition(7) = cellstr('0.00  0.00   0.00 0.00 0.00 0.00 1.00 0.00');
ActuatorToJointPosition(8) = cellstr('0.00  0.00   0.00 0.00 0.00 0.00 0.00 1.00');

for i = 1:8
    Row = docNode.createElement('Row');
    Row.setAttribute('Val', ActuatorToJointPosition(i));
    X_ActuatorToJointPosition.appendChild(Row);
end


% Coupling/JointToActuatorPosition
X_JointToActuatorPosition = docNode.createElement('JointToActuatorPosition');
X_Coupling.appendChild(X_JointToActuatorPosition);

JointToActuatorPosition = cell(8,1);
JointToActuatorPosition(1) = cellstr('1.00  0.00   0.00 0.00 0.00 0.00 0.00 0.00');
JointToActuatorPosition(2) = cellstr('0.00  1.00   0.00 0.00 0.00 0.00 0.00 0.00');
JointToActuatorPosition(3) = cellstr('0.00  1.00   1.00 0.00 0.00 0.00 0.00 0.00');
JointToActuatorPosition(4) = cellstr('0.00  0.00 0.6697 1.00 0.00 0.00 0.00 0.00');
JointToActuatorPosition(5) = cellstr('0.00  0.00   0.00 0.00 1.00 0.00 0.00 0.00');
JointToActuatorPosition(6) = cellstr('0.00  0.00   0.00 0.00 0.00 1.00 0.00 0.00');
JointToActuatorPosition(7) = cellstr('0.00  0.00   0.00 0.00 0.00 0.00 1.00 0.00');
JointToActuatorPosition(8) = cellstr('0.00  0.00   0.00 0.00 0.00 0.00 0.00 1.00');

for i = 1:8
    Row = docNode.createElement('Row');
    Row.setAttribute('Val', JointToActuatorPosition(i));
    X_JointToActuatorPosition.appendChild(Row);
end


% Coupling/ActuatorToJointTorque
X_ActuatorToJointTorque = docNode.createElement('ActuatorToJointTorque');
X_Coupling.appendChild(X_ActuatorToJointTorque);

ActuatorToJointTorque = cell(8,1);
ActuatorToJointTorque(1) = cellstr('1.00  0.00   0.00   0.00 0.00 0.00 0.00 0.00');
ActuatorToJointTorque(2) = cellstr('0.00  1.00   1.00   0.00 0.00 0.00 0.00 0.00');
ActuatorToJointTorque(3) = cellstr('0.00  0.00   1.00 0.6697 0.00 0.00 0.00 0.00');
ActuatorToJointTorque(4) = cellstr('0.00  0.00   0.00   1.00 0.00 0.00 0.00 0.00');
ActuatorToJointTorque(5) = cellstr('0.00  0.00   0.00   0.00 1.00 0.00 0.00 0.00');
ActuatorToJointTorque(6) = cellstr('0.00  0.00   0.00   0.00 0.00 1.00 0.00 0.00');
ActuatorToJointTorque(7) = cellstr('0.00  0.00   0.00   0.00 0.00 0.00 1.00 0.00');
ActuatorToJointTorque(8) = cellstr('0.00  0.00   0.00   0.00 0.00 0.00 0.00 1.00');

for i = 1:8
    Row = docNode.createElement('Row');
    Row.setAttribute('Val', ActuatorToJointTorque(i));
    X_ActuatorToJointTorque.appendChild(Row);
end

% Coupling/JointToActuatorTorque
X_JointToActuatorTorque = docNode.createElement('JointToActuatorTorque');
X_Coupling.appendChild(X_JointToActuatorTorque);

JointToActuatorTorque = cell(8,1);
JointToActuatorTorque(1) = cellstr('1.00  0.00   0.00    0.00 0.00 0.00 0.00 0.00');
JointToActuatorTorque(2) = cellstr('0.00  1.00  -1.00  0.6697 0.00 0.00 0.00 0.00');
JointToActuatorTorque(3) = cellstr('0.00  0.00   1.00 -0.6697 0.00 0.00 0.00 0.00');
JointToActuatorTorque(4) = cellstr('0.00  0.00   0.00    1.00 0.00 0.00 0.00 0.00');
JointToActuatorTorque(5) = cellstr('0.00  0.00   0.00    0.00 1.00 0.00 0.00 0.00');
JointToActuatorTorque(6) = cellstr('0.00  0.00   0.00    0.00 0.00 1.00 0.00 0.00');
JointToActuatorTorque(7) = cellstr('0.00  0.00   0.00    0.00 0.00 0.00 1.00 0.00');
JointToActuatorTorque(8) = cellstr('0.00  0.00   0.00    0.00 0.00 0.00 0.00 1.00');

for i = 1:8
    Row = docNode.createElement('Row');
    Row.setAttribute('Val', JointToActuatorTorque(i));
    X_JointToActuatorTorque.appendChild(Row);
end


% ---------- DigitalIn ---------------
Config.appendChild(docNode.createComment('Digital Input Configuration'));
% 2 boards
% for i = 0:1
%     % 3 sets NEG/POS/HOME
%     for j = 0:2
%         % 4 axes
%         for k = 0:3
%             DigitalIn = docNode.createElement('DigitalIn');
%             DigitalIn.setAttribute('BitID', num2str(k + 4*j));
%             DigitalIn.setAttribute('Name', strcat('MTML-D', num2str(j+3*i), num2str(k)));
%             DigitalIn.setAttribute('Name', strcat('MTML-D', num2str(j+3*i), num2str(k)));
%             DigitalIn.setAttribute('BoardID', num2str(boardID(i+1)));
%             DigitalIn.setAttribute('Pressed', '0');
%             DigitalIn.setAttribute('Trigger', 'all');
%             Config.appendChild(DigitalIn);
%         end
%     end
% end

% read from GUI
for b = 1:2
    for i = 1:12
        DigitalIn = docNode.createElement('DigitalIn');
        DigitalIn.setAttribute('BitID', num2str(i));
        DigitalIn.setAttribute('Name', 'hello' );
        DigitalIn.setAttribute('BoardID', num2str(boardID(b)));
        DigitalIn.setAttribute('Pressed', aDigital{i,3});
        DigitalIn.setAttribute('Trigger', 'all');
        Config.appendChild(DigitalIn);
    end
end

% generate xml file
xmlwrite(fileName,docNode);


%%
% delete temporary .m file
delete('temp.m');



end