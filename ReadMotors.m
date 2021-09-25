function q1 = ReadMotors(port_num)
%#codegen
coder.extrinsic('groupSyncWrite','groupSyncWriteAddParam','read2ByteTxRx');
coder.extrinsic('getLastRxPacketError','getLastTxRxResult');
coder.extrinsic('getTxRxResult','getRxPacketError','groupSyncWriteTxPacket');
coder.extrinsic('groupSyncWriteClearParam');

COMM_TX_FAIL                = -1001;        % Communication Tx Failed

dxl_error = 0;
dxl_comm_result = COMM_TX_FAIL; 

% Control table address
% ADDR_MX_GOAL_POSITION       = 30;
ADDR_MX_PRESENT_POSITION    = 36;

% Protocol version
PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel

COMM_SUCCESS                = 0;            % Communication Success result value

% % Initialize Groupsyncwrite instance
% group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

%% Set ID numbers of the the Dynamixels to be controlled:
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
    
%% Read present positions
    q = zeros(18,1);
    q(1) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID01, ADDR_MX_PRESENT_POSITION);
    q(2) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID02, ADDR_MX_PRESENT_POSITION);
    q(3) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID03, ADDR_MX_PRESENT_POSITION);
    q(4) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID04, ADDR_MX_PRESENT_POSITION);
    q(5) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID05, ADDR_MX_PRESENT_POSITION);
    q(6) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID06, ADDR_MX_PRESENT_POSITION);
    q(7) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID07, ADDR_MX_PRESENT_POSITION);
    q(8) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID08, ADDR_MX_PRESENT_POSITION);
    q(9) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID09, ADDR_MX_PRESENT_POSITION);
    q(10) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID10, ADDR_MX_PRESENT_POSITION);
    q(11) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID11, ADDR_MX_PRESENT_POSITION);
    q(12) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID12, ADDR_MX_PRESENT_POSITION);
    q(13) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID13, ADDR_MX_PRESENT_POSITION);
    q(14) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID14, ADDR_MX_PRESENT_POSITION);
    q(15) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID15, ADDR_MX_PRESENT_POSITION);
    q(16) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID16, ADDR_MX_PRESENT_POSITION);
    q(17) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID17, ADDR_MX_PRESENT_POSITION);
    q(18) = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID18, ADDR_MX_PRESENT_POSITION);

    q1 = jointangles(q);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end



