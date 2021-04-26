
% This script controls sagittal shoulders (1/2), frontal shoulders (3/4),
% elbows (5/6), transverse hips (7/8), frontal hips (9/10),
% sagittal hips (11/12), knees (13/14), sagittal ankles (15/16), and 
% frontal ankles (17/18) for TOBE. The joint numbers correspond to the
% Dynamixel ID numbers for the left/right joints, respectively.

% Go down to the third section of this script "Set goal positions, create
% command, present position variables" to change the number and values of
% goal positions to be executed when running the script. 

% NOTE: If you haven't already, go to 'Set Path' under the MATLAB Home Tab (Environment section),
% click 'Add with Subfolders' and select the c/include folder from the
% DynamixelSDK folder. Then, click 'Add Folder' and select the
% c/build/win64/output folder. Finally, click 'Add with Subfolders' and 
% select the matlab folder.

% Also, the plotting needs the functions 'Amat', 'Amat2', 'jointangles'.
% If you don't need the plotting, just delete that section.

clc;
clear;

% settings:
% find COM port number under 'Control Panel', 'Hardware and Sound',
% 'Devices and Printers', 'Device Manager', 'Ports'
% If the USB2Dynamixel is plugged into the computer and there is a solid
% red light, it should show up as a USB device under 'Ports'.
DEVICENAME                  = 'COM4';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'

%% Set Dynamixels and joint limits
% set ID numbers of the the Dynamixels to be controlled:
ID01                     = 1;            % Dynamixel IDs for sagittal shoulders
ID02                     = 2;
ID03                     = 3;            % Dynamixel IDs for frontal shoulders
ID04                     = 4;
ID05                     = 5;            % Dynamixel IDs for elbows
ID06                     = 6;
ID07                     = 7;            % Dynamixel IDs for transverse hips
ID08                     = 8;
ID09                     = 9;            % Dynamixel IDs for frontal hips
ID10                     = 10;
ID11                     = 11;            % Dynamixel IDs for sagittal hips
ID12                     = 12;
ID13                     = 13;            % Dynamixel IDs for knees
ID14                     = 14;
ID15                     = 15;            % Dynamixel IDs for sagittal ankles
ID16                     = 16;
ID17                     = 17;            % Dynamixel IDs for frontal ankles
ID18                     = 18;

% set joint home-/mid-points
MID01  = 200;          % R sag. shoulder: +/- = forward/backward
MID02  = 800;          % L sag. shoulder: +/- = backward/forward
MID03  = 300;          % R fro. shoulder: +/- = outward/inward 
MID04  = 720;          % L fro. shoulder: +/- = inward/outward 
MID05  = 450;          % R elbow: +/- = extended/bent 
MID06  = 570;          % L elbow: +/- = bent/extended
MID07  = 510;          % R transverse hip: +/- = outward/inward 
MID08  = 510;          % L transverse hip: +/- = inward/outward
MID09  = 520;          % R fro. hip: +/- = outward/inward 
MID10  = 500;          % L fro. hip: +/- = inward/outward 
MID11  = 510;          % R sag. hip: +/- = forward/backward
MID12  = 510;          % L sag. hip: +/- = backward/forward
MID13  = 470;          % R knee: +/- = extended/bent
MID14  = 550;          % L knee: +/- = bent/extended 
MID15  = 510;          % R sag. ankle: +/- = toes down/up
MID16  = 510;          % L sag. ankle: +/- = toes up/down 
MID17  = 510;          % R fro. ankle: +/- = outer foot turned down/up
MID18  = 510;          % L fro. ankle: +/- = outer foot turned up/down

%% Set up library and define default parameters
ESC_CHARACTER               = 'e';          % Key for escaping loop

% set lib_name:
lib_name = '';
if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

% Control table address
ADDR_MX_TORQUE_ENABLE       = 24;           % Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30;
ADDR_MX_PRESENT_POSITION    = 36;

% Protocol version
PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel

% Default settings:
BAUDRATE                    = 1000000;

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10;           % Dynamixel moving status threshold

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

dxl_comm_result = COMM_TX_FAIL;             % Communication result
dxl_error = 0;                              % Dynamixel error

%% Set goal positions, create command, present position variables
index = 1;

mid = [MID01;MID02;MID03;MID04;MID05;MID06;MID07;MID08;MID09;MID10;MID11;MID12;MID13;MID14;MID15;MID16;MID17;MID18];
dx0 = [zeros(8,1);20;-20;25;-25;-70;70;-70;70;20;-20];
home = mid + dx0; % "home" position

% adjustments for leaning toward right leg
dx1 = zeros(18,1);
dx1(9) = -5; dx1(10) = -5;
dx1(11) = 5; dx1(12) = 15;
dx1(13) = -10; dx1(14) = -10;
dx1(15) = -10; dx1(16) = -10;
dx1(17) = -10; dx1(18) = -10;

% adjustments for leaning toward left leg 
dx2 = zeros(18,1);
dx2(9) = 5; dx2(10) = 5;
dx2(11) = -15; dx2(12) = -5;
dx2(13) = 10; dx2(14) = 10;
dx2(15) = 10; dx2(16) = 10;
dx2(17) = 10; dx2(18) = 10;

LR1 = home + dx1; % R position one: slight lean to right
LR2 = LR1 + dx1; % R position two: more lean to right
LR3 = LR2 + dx1; % R position three: most weight on right stance foot
LL1 = home + dx2; % L position one: slight lean to left
LL2 = LL1 + dx2; % L position two: more lean to left
LL3 = LL2 + dx2; % L position three: most weight on left stance foot

% test motion sequence: 
qdes = [home,LR1,LR2,LR3,LR2,LR1,home,LL1,LL2,LL3,LL2,LL1,home];
np = size(qdes,2); % number of goal positions

%% Open port and set baudrate
% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

%% Enable Dynamixel Torques and connect Dynamixels
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID01, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID02, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID03, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID04, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID05, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID06, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID07, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID08, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID09, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID10, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID11, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID12, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID13, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID14, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID15, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID16, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID17, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID18, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

%% IMU Calibration

% calibrate and get acceleration data from BNO055 sensor connected to Arduino Nano:
IMUport = 'COM3'; % find USB port that Arduino is connected to
a = arduino(IMUport,'Nano3',"Libraries","Adafruit/BNO055") % Arduino object
b = addon(a,'Adafruit/BNO055') % BNO055 sensor object

tic; 
while (toc < 90)
    [status,~] = readCalibrationStatus(b) % write calibration status to Command Window
    if strcmpi(status.Accelerometer,'full')
    % calibrate accelerometer by holding sensor in different angles every few seconds: 
    % e.g., rotate 45 degrees about x-axis every 5 seconds until back to initial orientation,
    % then rotate 45 degrees about y-axis every 5 seconds until back to initial orientation
        break; %Accelerometer is calibrated proceed further
    end
    pause(10);
end

input('Accelerometer is calibrated. Attach IMU to TOBE, then press any key to continue!\n', 's')

%% JOINT CONTROL LOOP 
time = 0; 
elapsed = 0;
interval = 2; 
t_end = 35;

while time <= t_end 
    if time == 0
        if input('Press any key to begin test motion! (or input e to quit!)\n', 's') == ESC_CHARACTER
            break;
        end
    end
    
    tic
    % Set goal position to current command
    CMD01 = qdes(1,index);
    CMD02 = qdes(2,index);
    CMD03 = qdes(3,index); 
    CMD04 = qdes(4,index);
    CMD05 = qdes(5,index);
    CMD06 = qdes(6,index);   
    CMD07 = qdes(7,index);   
    CMD08 = qdes(8,index);    
    CMD09 = qdes(9,index); 
    CMD10 = qdes(10,index);   
    CMD11 = qdes(11,index);
    CMD12 = qdes(12,index); 
    CMD13 = qdes(13,index); 
    CMD14 = qdes(14,index);
    CMD15 = qdes(15,index);
    CMD16 = qdes(16,index);   
    CMD17 = qdes(17,index);   
    CMD18 = qdes(18,index);    
    
    % send command positions to motors
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID01, ADDR_MX_GOAL_POSITION, CMD01);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID02, ADDR_MX_GOAL_POSITION, CMD02);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID03, ADDR_MX_GOAL_POSITION, CMD03);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID04, ADDR_MX_GOAL_POSITION, CMD04);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID05, ADDR_MX_GOAL_POSITION, CMD05);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID06, ADDR_MX_GOAL_POSITION, CMD06);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID07, ADDR_MX_GOAL_POSITION, CMD07);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID08, ADDR_MX_GOAL_POSITION, CMD08);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID09, ADDR_MX_GOAL_POSITION, CMD09);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID10, ADDR_MX_GOAL_POSITION, CMD10);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID11, ADDR_MX_GOAL_POSITION, CMD11);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID12, ADDR_MX_GOAL_POSITION, CMD12);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID13, ADDR_MX_GOAL_POSITION, CMD13);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID14, ADDR_MX_GOAL_POSITION, CMD14);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID15, ADDR_MX_GOAL_POSITION, CMD15);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID16, ADDR_MX_GOAL_POSITION, CMD16);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID17, ADDR_MX_GOAL_POSITION, CMD17);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID18, ADDR_MX_GOAL_POSITION, CMD18);
    
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    
    % Read present position
    NOW01 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID01, ADDR_MX_PRESENT_POSITION);
    NOW02 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID02, ADDR_MX_PRESENT_POSITION);
    NOW03 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID03, ADDR_MX_PRESENT_POSITION);
    NOW04 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID04, ADDR_MX_PRESENT_POSITION);
    NOW05 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID05, ADDR_MX_PRESENT_POSITION);
    NOW06 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID06, ADDR_MX_PRESENT_POSITION);
    NOW07 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID07, ADDR_MX_PRESENT_POSITION);
    NOW08 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID08, ADDR_MX_PRESENT_POSITION);
    NOW09 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID09, ADDR_MX_PRESENT_POSITION);
    NOW10 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID10, ADDR_MX_PRESENT_POSITION);
    NOW11 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID11, ADDR_MX_PRESENT_POSITION);
    NOW12 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID12, ADDR_MX_PRESENT_POSITION);
    NOW13 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID13, ADDR_MX_PRESENT_POSITION);
    NOW14 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID14, ADDR_MX_PRESENT_POSITION);
    NOW15 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID15, ADDR_MX_PRESENT_POSITION);
    NOW16 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID16, ADDR_MX_PRESENT_POSITION);
    NOW17 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID17, ADDR_MX_PRESENT_POSITION);
    NOW18 = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID18, ADDR_MX_PRESENT_POSITION);

    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    if elapsed >= interval
        index = index + 1; % increase index to go to next goal position during next loop
        elapsed = 0;
    end
    
    if index > np % break loop after last goal position is reached
        break;
    end
    
    % PLOTTING: 
    
    % Compute robot joint positions
    ang = jointangles([NOW01;NOW02;NOW03;NOW04;NOW05;NOW06;NOW07;NOW08;NOW09;NOW10;NOW11;NOW12;NOW13;NOW14;NOW15;NOW16;NOW17;NOW18]);
    
    % Lower body:
    % Values 1-2/9-10: left/right foot, 3/8: L/R ankle, 4/7: L/R knee, 5/6: L/R hip
    
    % set stance foot:
    
    if index > 7
        stance = 0;
        lowerX = [ALT(1,4) ALF(1,4)];
        lowerY = [ALT(2,4) ALF(2,4)];
        lowerZ = [ALT(3,4) ALF(3,4)];        
    else
        stance = 1;
        lowerX = [0 0];
        lowerY = [0 0];
        lowerZ = [50 0];
    end

    % Find lower body joints
    if(stance == 0) 
        A0 = ALF*Amat(0,0,30,0); 
        A1 = A0*Amat(ang(1),0,45,90);
        A2 = A1*Amat(ang(2)-12.680,15,62,0);
        A3 = A2*Amat(ang(3)+26.294,0,66,0);    
        A5 = A3*Amat(ang(4)-13.614,0,70,90)*Amat(ang(5)-90,0,0,-90);
        AB = A5*Amat(ang(6),47,38.5,0);
        A6 = AB*Amat(0,-47,38.5,0);
        A8 = A6*Amat(ang(7),0,0,-90)*Amat(ang(8)+90,0,70,90);
        A9 = A8*Amat(ang(9)-13.614,0,66,0);
        A10 = A9*Amat(ang(10)+26.294,-15,62,0); 
        A11 = A10*Amat(ang(11)-12.680,0,45,-90);
        ARF = A11*Amat(ang(12)+180,0,-30,0);
        ART = ARF*Amat(0,50,0,0);

        lowerX(3) = A0(1,4); lowerY(3) = A0(2,4); lowerZ(3) = A0(3,4);
        lowerX(4) = A1(1,4); lowerY(4) = A1(2,4); lowerZ(4) = A1(3,4);
        lowerX(5) = A2(1,4); lowerY(5) = A2(2,4); lowerZ(5) = A2(3,4);
        lowerX(6) = A3(1,4); lowerY(6) = A3(2,4); lowerZ(6) = A3(3,4);
        lowerX(7) = A5(1,4); lowerY(7) = A5(2,4); lowerZ(7) = A5(3,4);
        lowerX(8) = AB(1,4); lowerY(8) = AB(2,4); lowerZ(8) = AB(3,4);
        lowerX(9) = A6(1,4); lowerY(9) = A6(2,4); lowerZ(9) = A6(3,4);
        lowerX(10) = A8(1,4); lowerY(10) = A8(2,4); lowerZ(10) = A8(3,4);
        lowerX(11) = A9(1,4); lowerY(11) = A9(2,4); lowerZ(11) = A9(3,4);
        lowerX(12) = A10(1,4); lowerY(12) = A10(2,4); lowerZ(12) = A10(3,4);
        lowerX(13) = A11(1,4); lowerY(13) = A11(2,4); lowerZ(13) = A11(3,4);
        lowerX(14) = ARF(1,4); lowerY(14) = ARF(2,4); lowerZ(14) = ARF(3,4);
        lowerX(15) = ART(1,4); lowerY(15) = ART(2,4); lowerZ(15) = ART(3,4);
    else
        ang(1:12) = -ang(1:12);
        A11 = Amat2(ang(12)+180,0,30,0);
        A10 = A11*Amat2(ang(11)+12.680,0,-45,90);
        A9 = A10*Amat2(ang(10)-26.294,15,-62,0);
        A8 = A9*Amat2(ang(9)+13.614,0,-66,0);
        A6 = A8*Amat2(ang(8)-90,0,-70,-90)*Amat2(ang(7),0,0,90);
        AB = A6*Amat2(0,47,-38.5,0); 
        A5 = AB*Amat2(ang(6),-47,-38.5,0);
        A3 = A5*Amat2(ang(5)+90,0,0,90)*Amat2(ang(4)+13.614,0,-70,-90);
        A2 = A3*Amat2(ang(3)-26.294,0,-66,0);
        A1 = A2*Amat2(ang(2)+12.680,-15,-62,0);
        A0 = A1*Amat2(ang(1),0,-45,-90);
        ALF = A0*Amat2(0,0,-30,0);
        ALT = ALF*Amat2(0,50,0,0);

        lowerX(3) = A11(1,4); lowerY(3) = A11(2,4); lowerZ(3) = A11(3,4);
        lowerX(4) = A10(1,4); lowerY(4) = A10(2,4); lowerZ(4) = A10(3,4);
        lowerX(5) = A9(1,4); lowerY(5) = A9(2,4); lowerZ(5) = A9(3,4);
        lowerX(6) = A8(1,4); lowerY(6) = A8(2,4); lowerZ(6) = A8(3,4);
        lowerX(7) = A6(1,4); lowerY(7) = A6(2,4); lowerZ(7) = A6(3,4);
        lowerX(8) = AB(1,4); lowerY(8) = AB(2,4); lowerZ(8) = AB(3,4);
        lowerX(9) = A5(1,4); lowerY(9) = A5(2,4); lowerZ(9) = A5(3,4);
        lowerX(10) = A3(1,4); lowerY(10) = A3(2,4); lowerZ(10) = A3(3,4);
        lowerX(11) = A2(1,4); lowerY(11) = A2(2,4); lowerZ(11) = A2(3,4);
        lowerX(12) = A1(1,4); lowerY(12) = A1(2,4); lowerZ(12) = A1(3,4);
        lowerX(13) = A0(1,4); lowerY(13) = A0(2,4); lowerZ(13) = A0(3,4);
        lowerX(14) = ALF(1,4); lowerY(14) = ALF(2,4); lowerZ(14) = ALF(3,4);
        lowerX(15) = ALT(1,4); lowerY(15) = ALT(2,4); lowerZ(15) = ALT(3,4);
    end

    % Upper body:

    A12 = AB*Amat(0,71,-38.5,0)*Amat(90,0,0,-90);
    A13 = A12*Amat(ang(13)+90,27,13,90);
    A14 = A13*Amat(ang(14),0,67,-90);
    ALW = A14*Amat(ang(15),0,50,0);
    ALFT = ALW*Amat(0,0,59,0);

    upperXL(1) = AB(1,4); upperYL(1) = AB(2,4); upperZL(1) = AB(3,4);
    upperXL(2) = A12(1,4); upperYL(2) = A12(2,4); upperZL(2) = A12(3,4);
    upperXL(3) = A13(1,4); upperYL(3) = A13(2,4); upperZL(3) = A13(3,4);
    upperXL(4) = A14(1,4); upperYL(4) = A14(2,4); upperZL(4) = A14(3,4);
    upperXL(5) = ALW(1,4); upperYL(5) = ALW(2,4); upperZL(5) = ALW(3,4);
    upperXL(6) = ALFT(1,4); upperYL(6) = ALFT(2,4); upperZL(6) = ALFT(3,4);

    A15 = AB*Amat(0,71,38.5,0)*Amat(90,0,0,90);
    A16 = A15*Amat(ang(16)-90,27,13,-90);
    A17 = A16*Amat(ang(17),0,67,90);
    ARW = A17*Amat(ang(18),0,50,0);
    ARFT = ARW*Amat(0,0,59,0);

    upperXR(1) = AB(1,4); upperYR(1) = AB(2,4); upperZR(1) = AB(3,4);
    upperXR(2) = A15(1,4); upperYR(2) = A15(2,4); upperZR(2) = A15(3,4);
    upperXR(3) = A16(1,4); upperYR(3) = A16(2,4); upperZR(3) = A16(3,4);
    upperXR(4) = A17(1,4); upperYR(4) = A17(2,4); upperZR(4) = A17(3,4);
    upperXR(5) = ARW(1,4); upperYR(5) = ARW(2,4); upperZR(5) = ARW(3,4);
    upperXR(6) = ARFT(1,4); upperYR(6) = ARFT(2,4); upperZR(6) = ARFT(3,4);
    
    % Graphical representation:
    cla
    plot3(lowerZ,lowerY,lowerX,':bs','LineWidth',3,'MarkerEdgeColor','k','MarkerFaceColor',[1 0 0],'MarkerSize',5)
    hold on
    plot3(upperZL,upperYL,upperXL,':ro','LineWidth',3,'MarkerEdgeColor','k','MarkerFaceColor',[1 0 0],'MarkerSize',5)
    plot3(upperZR,upperYR,upperXR,':ro','LineWidth',3,'MarkerEdgeColor','k','MarkerFaceColor',[1 0 0],'MarkerSize',5)

    xlabel('Z')
    ylabel('Y')
    zlabel('X')
    view([-132,-10])
    daspect([1 1 1])

    zlim([-10 450])
    ylim([-250 250])
    xlim([-100 100])
    title('Robot Depiction')

    %Set figure size
    set(gcf,'Position', [50, 50, 600, 650]);

    drawnow();
    % END of PLOTTING
    
    a = toc;
    time = time + a;
    elapsed = elapsed + a;
    
    % read x-, y-, z-axis accelerations from IMU, output to Cmd Window
    [data,~] = readAcceleration(b) 
    
end

%% Disable Dynamixel Torque and close program
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID01, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID02, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID03, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID04, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID05, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID06, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID07, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID08, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID09, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID10, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID11, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID12, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID13, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID14, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID15, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID16, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID17, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, ID18, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

% close all;
% clear;



