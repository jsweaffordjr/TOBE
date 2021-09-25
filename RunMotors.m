function anow = RunMotors(port_num,group_num,CMD)
%#codegen
coder.extrinsic('groupSyncWrite','groupSyncWriteAddParam','read2ByteTxRx');
coder.extrinsic('getLastRxPacketError','getLastTxRxResult');
coder.extrinsic('getTxRxResult','getRxPacketError','groupSyncWriteTxPacket');
coder.extrinsic('groupSyncWriteClearParam','write1ByteTxRx');


COMM_TX_FAIL                = -1001;        % Communication Tx Failed
dxl_comm_result = COMM_TX_FAIL;

anow = [1;1];

% Control table address

% Data Byte Length
LEN_MX_GOAL_POSITION        = 2;

% Protocol version
PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel

COMM_SUCCESS                = 0;            % Communication Success result value
ADDR_MX_TORQUE_ENABLE       = 24;           % Control table address is different in Dynamixel model

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque

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
    
%% Send command positions to motors 
    dxl_addparam_result = 1;
    
    % Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID01, CMD(1), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID01: groupSyncWrite addparam failed');
        return;
    end

    % Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID02, CMD(2), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID02: groupSyncWrite addparam failed');
        return;
    end
 
    % Add Dynamixel#3 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID03, CMD(3), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID03: groupSyncWrite addparam failed');
        return;
    end

    % Add Dynamixel#4 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID04, CMD(4), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID04: groupSyncWrite addparam failed');
        return;
    end
     
    % Add Dynamixel#5 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID05, CMD(5), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID05: groupSyncWrite addparam failed');
        return;
    end

    % Add Dynamixel#6 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID06, CMD(6), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID06: groupSyncWrite addparam failed');
        return;
    end
 
    % Add Dynamixel#7 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID07, CMD(7), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID07: groupSyncWrite addparam failed');
        return;
    end

    % Add Dynamixel#8 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID08, CMD(8), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID08: groupSyncWrite addparam failed');
        return;
    end
 
    % Add Dynamixel#9 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID09, CMD(9), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID09: groupSyncWrite addparam failed');
        return;
    end

    % Add Dynamixel#10 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID10, CMD(10), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID10: groupSyncWrite addparam failed');
        return;
    end
     
    % Add Dynamixel#11 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID11, CMD(11), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID11: groupSyncWrite addparam failed');
        return;
    end

    % Add Dynamixel#12 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID12, CMD(12), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID12: groupSyncWrite addparam failed');
        return;
    end
 
    % Add Dynamixel#13 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID13, CMD(13), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID13: groupSyncWrite addparam failed');
        return;
    end

    % Add Dynamixel#14 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID14, CMD(14), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID14: groupSyncWrite addparam failed');
        return;
    end
 
    % Add Dynamixel#15 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID15, CMD(15), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID15: groupSyncWrite addparam failed');
        return;
    end

    % Add Dynamixel#16 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, ID16, CMD(16), LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= 1
        fprintf('ID16: groupSyncWrite addparam failed');
        return;
    end
     
%     if anks(1) == 1 % if ankle is supposed to be on
%         if aprev(1) ~= 1 % if it was turned off previously,
%             % turn it back on:
%             write1ByteTxRx(port_num, PROTOCOL_VERSION, ID17, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
%         end
        % update the joint angle position
        % Add Dynamixel#17 goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWriteAddParam(group_num, ID17, CMD(17), LEN_MX_GOAL_POSITION);
        if dxl_addparam_result ~= 1
            fprintf('ID17: groupSyncWrite addparam failed');
            return;
        end
%     else % if the ankle is supposed to be off
%         anow(1) = 0;
%         if aprev(1) == 1 % if it was on previously,
%             % turn it off:
%             write1ByteTxRx(port_num, PROTOCOL_VERSION, ID17, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
%         end
%     end
%     
%     if anks(2) == 1 % if left ankle is supposed to be on,
%         if aprev(2) ~= 1 % if it was off previously,
%             % turn it back on:
%             write1ByteTxRx(port_num, PROTOCOL_VERSION, ID18, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
%         end
        % update the joint angle position
        % Add Dynamixel#18 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteAddParam(group_num, ID18, CMD(18), LEN_MX_GOAL_POSITION);
        if dxl_addparam_result ~= 1
            fprintf('ID18: groupSyncWrite addparam failed');
            return;
        end
%     else % if left ankle is supposed to be off,
%         anow(2) = 0;
%         if aprev(2) == 1 % if it was on previously,
%             % turn it off
%             write1ByteTxRx(port_num, PROTOCOL_VERSION, ID18, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
%         end
%     end
    
    % Syncwrite goal position
    groupSyncWriteTxPacket(group_num);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    end

    % Clear syncwrite parameter storage
    groupSyncWriteClearParam(group_num);



