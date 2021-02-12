
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

clc;
clear;

% settings:
% find COM port number under 'Control Panel', 'Hardware and Sound',
% 'Devices and Printers', 'Device Manager', 'Ports'
% If the USB2Dynamixel is plugged into the computer and there is a solid
% red light, it should show up as a USB device under 'Ports'.
DEVICENAME                  = 'COM3';       % Check which port is being used on your controller
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

% set joint limits
MIN01  = 0;            % right arm swung backward maximally
MID01  = 200;          % "home position" (arm down at side)
MAX01  = 500;          % arm swung out in front maximally

MIN02  = 500;          % left arm swung in front maximally
MID02  = 800;          % "home position" (arm down at side)
MAX02  = 1000;         % arm swung out backward maximally

MIN03  = 220;          % right upper arm swung inward maximally
MID03  = 300;          % "home position" 
MAX03  = 510;          % upper arm swung outward maximally 

MIN04  = 510;          % left upper arm swung outward maximally
MID04  = 720;          % "home position" 
MAX04  = 800;          % upper arm swung inward maximally

MIN05  = 230;          % right elbow extended maximally
MID05  = 450;          % "home position" 
MAX05  = 510;          % elbow bent maximally

MIN06  = 510;         % left elbow extended maximally 
MID06  = 570;         % "home position" 
MAX06  = 790;         % elbow bent maximally

MIN07  = 470;         % right leg turned inward maximally 
MID07  = 510;         % "home position" 
MAX07  = 550;         % leg turned outward maximally 

MIN08  = 470;         % left leg turned outward maximally 
MID08  = 510;         % "home position" 
MAX08  = 550;         % leg turned inward maximally

MIN09  = 490;          % right frontal hip swung inward maximally
MID09  = 510;          % "home position" 
MAX09  = 520;          % hip swung outward maximally (hard physical limit)

MIN10  = 500;          % left frontal hip swung outward maximally (hard physical limit)
MID10  = 510;          % "home position" 
MAX10  = 530;          % hip swung inward maximally

MIN11  = 300;          % right sagittal hip swung backward maximally
MID11  = 510;          % "home position" 
MAX11  = 850;          % hip swung forward maximally

MIN12  = 120;          % left sagittal hip swung forward maximally
MID12  = 510;          % "home position" 
MAX12  = 730;          % hip swung backward maximally

MIN13  = 220;          % right knee bent maximally
MID13  = 470;          % "home position" 
MAX13  = 510;          % knee extended maximally (hard physical limit)

MIN14  = 510;          % left knee extended maximally (hard physical limit)
MID14  = 550;          % "home position" 
MAX14  = 800;          % knee bent maximally

MIN15  = 400;          % right sagittal ankle, toes up maximally
MID15  = 510;          % "home position" 
MAX15  = 600;          % sagittal ankle, toes down maximally

MIN16  = 420;         % left sagittal ankle, toes down maximally 
MID16  = 510;         % "home position" 
MAX16  = 620;         % sagittal ankle, toes up maximally

MIN17  = 470;         % right frontal ankle turned up maximally (hard physical limit)
MID17  = 510;         % "home position" 
MAX17  = 600;         % frontal ankle turned down maximally 

MIN18  = 420;         % left frontal ankle turned down maximally 
MID18  = 510;         % "home position" 
MAX18  = 550;         % frontal ankle turned up maximally (hard physical limit)

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
np = 3; % number of goal positions
index = 1;
GOAL01 = MID01*ones(1,np);                   % Goal positions for sagittal shoulders
GOAL02 = MID02*ones(1,np);
GOAL03 = MID03*ones(1,np);                   % Goal positions for frontal shoulders
GOAL04 = MID04*ones(1,np);
GOAL05 = MID05*ones(1,np);                   % Goal positions for elbows
GOAL06 = MID06*ones(1,np);
GOAL07 = MID07*ones(1,np);                   % Goal positions for transverse hips
GOAL08 = MID08*ones(1,np);
GOAL09 = MID09*ones(1,np);                   % Goal positions for frontal hips
GOAL10 = MID10*ones(1,np);
GOAL11 = MID11*ones(1,np);                   % Goal positions for sagittal hips
GOAL12 = MID12*ones(1,np);
GOAL13 = MID13*ones(1,np);                   % Goal positions for knees
GOAL14 = MID14*ones(1,np);
GOAL15 = MID15*ones(1,np);                   % Goal positions for sagittal ankles
GOAL16 = MID16*ones(1,np);
GOAL17 = [MID17,MAX17,MID17]; %MID17*ones(1,np);                   % Goal positions for frontal ankles
GOAL18 = [MID18,MAX18,MID18]; %MID18*ones(1,np);

CMD01 = GOAL01(index);                   % Command positions for sagittal shoulders
CMD02 = GOAL02(index); 
CMD03 = GOAL03(index);                   % Command positions for frontal shoulders
CMD04 = GOAL04(index);
CMD05 = GOAL05(index);                   % Command positions for elbows
CMD06 = GOAL06(index);
CMD07 = GOAL07(index);                   % Command positions for transverse hips
CMD08 = GOAL08(index);
CMD09 = GOAL09(index);                   % Command positions for frontal hips
CMD10 = GOAL10(index);
CMD11 = GOAL11(index);                   % Command positions for sagittal hips
CMD12 = GOAL12(index);
CMD13 = GOAL13(index);                   % Command positions for knees
CMD14 = GOAL14(index); 
CMD15 = GOAL15(index);                   % Command positions for sagittal ankles
CMD16 = GOAL16(index);
CMD17 = GOAL17(index);                 % Command positions for frontal ankles
CMD18 = GOAL18(index);

NOW01 = 0;                      % Present positions for sagittal shoulders
NOW02 = 0;
NOW03 = 0;                      % Present positions for frontal shoulders
NOW04 = 0;
NOW05 = 0;                      % Present positions for elbows
NOW06 = 0;
NOW07 = 0;                      % Present positions for transverse hips
NOW08 = 0;
NOW09 = 0;                      % Present positions for frontal hips
NOW10 = 0;
NOW11 = 0;                      % Present positions for sagittal hips
NOW12 = 0;
NOW13 = 0;                      % Present positions for knees
NOW14 = 0;
NOW15 = 0;                      % Present positions for sagittal ankles
NOW16 = 0;
NOW17 = 0;                     % Present positions for frontal ankles
NOW18 = 0;

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

%% JOINT CONTROL LOOP

while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end

    % Set goal position to current command
    CMD01 = GOAL01(index);
    CMD02 = GOAL02(index);
    CMD03 = GOAL03(index); 
    CMD04 = GOAL04(index);
    CMD05 = GOAL05(index);
    CMD06 = GOAL06(index);   
    CMD07 = GOAL07(index);   
    CMD08 = GOAL08(index);    
    CMD09 = GOAL09(index); 
    CMD10 = GOAL10(index);   
    CMD11 = GOAL11(index);
    CMD12 = GOAL12(index); 
    CMD13 = GOAL13(index); 
    CMD14 = GOAL14(index);
    CMD15 = GOAL15(index);
    CMD16 = GOAL16(index);   
    CMD17 = GOAL17(index);   
    CMD18 = GOAL18(index);    
    
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

    count = 0;
    while count < 10
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

        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID01, GOAL01(index), NOW01);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID02, GOAL02(index), NOW02);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID03, GOAL03(index), NOW03);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID04, GOAL04(index), NOW04);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID05, GOAL05(index), NOW05);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID06, GOAL06(index), NOW06);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID07, GOAL07(index), NOW07);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID08, GOAL08(index), NOW08);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID09, GOAL09(index), NOW09);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID10, GOAL10(index), NOW10);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID11, GOAL11(index), NOW11);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID12, GOAL12(index), NOW12);        
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID13, GOAL13(index), NOW13);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID14, GOAL14(index), NOW14);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID15, GOAL15(index), NOW15);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID16, GOAL16(index), NOW16);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID17, GOAL17(index), NOW17);
        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', ID18, GOAL18(index), NOW18);
        
        % break loop when any of the motors is within threshold of desired position
        if ~(abs(GOAL01(index) - NOW01) > DXL_MOVING_STATUS_THRESHOLD)
            if ~(abs(GOAL02(index) - NOW02) > DXL_MOVING_STATUS_THRESHOLD)
                if ~(abs(GOAL03(index) - NOW03) > DXL_MOVING_STATUS_THRESHOLD)
                    if ~(abs(GOAL04(index) - NOW04) > DXL_MOVING_STATUS_THRESHOLD)
                        if ~(abs(GOAL05(index) - NOW05) > DXL_MOVING_STATUS_THRESHOLD)
                            if ~(abs(GOAL06(index) - NOW06) > DXL_MOVING_STATUS_THRESHOLD)
                                if ~(abs(GOAL07(index) - NOW07) > DXL_MOVING_STATUS_THRESHOLD)
                                    if ~(abs(GOAL08(index) - NOW08) > DXL_MOVING_STATUS_THRESHOLD)
                                        if ~(abs(GOAL09(index) - NOW09) > DXL_MOVING_STATUS_THRESHOLD)
                                            if ~(abs(GOAL10(index) - NOW10) > DXL_MOVING_STATUS_THRESHOLD)
                                                if ~(abs(GOAL11(index) - NOW11) > DXL_MOVING_STATUS_THRESHOLD)
                                                    if ~(abs(GOAL12(index) - NOW12) > DXL_MOVING_STATUS_THRESHOLD)
                                                        if ~(abs(GOAL13(index) - NOW13) > DXL_MOVING_STATUS_THRESHOLD)
                                                            if ~(abs(GOAL14(index) - NOW14) > DXL_MOVING_STATUS_THRESHOLD)
                                                                if ~(abs(GOAL15(index) - NOW15) > DXL_MOVING_STATUS_THRESHOLD)
                                                                    if ~(abs(GOAL16(index) - NOW16) > DXL_MOVING_STATUS_THRESHOLD)
                                                                        if ~(abs(GOAL17(index) - NOW17) > DXL_MOVING_STATUS_THRESHOLD)
                                                                            if ~(abs(GOAL18(index) - NOW18) > DXL_MOVING_STATUS_THRESHOLD)                            
                                                                                break;  
                                                                            end
                                                                        end
                                                                    end
                                                                end
                                                            end
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        count = count + 1;
    end
    index = index + 1; % increase index to go to next goal position during next loop
    if index > np % break loop after last goal position is reached
        break;
    end
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

close all;
% clear;

