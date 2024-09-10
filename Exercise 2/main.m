function main()     
addpath('./simulation_scripts');
clc;
clear;
close all
real_robot = false;
%% Initialization - DON'T CHANGE ANYTHING from HERE ... 
% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 15;
loop = 1;
maxloops = ceil(end_time/deltat);
mission.phase = 1;
mission.phase_time = 0;
model = load("panda.mat");

% UDP Connection with Franka Interface
if real_robot == true
    hudprLeft = dsp.UDPReceiver('LocalIPPort',1501,'MaximumMessageLength',255);
    hudprRight = dsp.UDPReceiver('LocalIPPort',1503,'MaximumMessageLength',255);
    hudpsLeft = dsp.UDPSender('RemoteIPPort',1500);
    hudpsLeft.RemoteIPAddress = '127.0.0.1';
    hudpsRight = dsp.UDPSender('RemoteIPPort',1502);
    hudpsRight.RemoteIPAddress = '127.0.0.1';
else
    hudps = dsp.UDPSender('RemoteIPPort',1505);
    hudps.RemoteIPAddress = '127.0.0.1';
end
%% ... to HERE.
% Init robot model
wTb_left = eye(4);                         % fixed transformation word -> base1
right_arm_base_position = [1.06 -0.01 0]';
wTb_right = [rotation(0,0,pi) right_arm_base_position; 0 0 0 1]; % fixed transformation word -> base2

plt = InitDataPlot(maxloops);
pandaArms = InitRobot(model,wTb_left,wTb_right);
% Init object and tools frames
obj_length = 0.1;
w_obj_pos = [0.5 0 0.59]';
w_obj_ori = rotation(0,0,0);
pandaArms.ArmL.wTo = [w_obj_ori w_obj_pos; 0 0 0 1];
pandaArms.ArmR.wTo = pandaArms.ArmL.wTo;

theta = deg2rad(-44.9949);    % FIXED ANGLE BETWEEN EE AND TOOL 
tool_length = 0.2104;         % FIXED DISTANCE BETWEEN EE AND TOOL

% Define transformation matrix from ee to tool.
e_tool_pos = [0 0 tool_length]';
e_tool_ori = rotation(0,0,theta);
pandaArms.ArmL.eTt = [e_tool_ori e_tool_pos; 0 0 0 1];
pandaArms.ArmR.eTt = pandaArms.ArmL.eTt;

% Transformation matrix from <t> to <w>
pandaArms.ArmL.wTt = pandaArms.ArmL.wTe * pandaArms.ArmL.eTt;
pandaArms.ArmR.wTt = pandaArms.ArmR.wTe * pandaArms.ArmR.eTt;

%% Defines the goal position for the end-effector/tool position task
% First goal reach the grasping points (Reach the object)
theta_2 = deg2rad(30);
w_O_grasp_pos_left = w_obj_pos - [obj_length/2 0 0]';
w_O_grasp_pos_right = w_obj_pos + [obj_length/2 0 0]';

pandaArms.ArmL.wTg = [pandaArms.ArmL.wTt(1:3,1:3) * rotation(0,theta_2,0), w_O_grasp_pos_left; 0 0 0 1];
pandaArms.ArmR.wTg = [pandaArms.ArmR.wTt(1:3,1:3) * rotation(0,theta_2,0), w_O_grasp_pos_right; 0 0 0 1];

% Second goal move the object
w_g_pos = [0.65 -0.35 0.28]';
w_g_ori = rotation(0,0,0);                   % identity matrix 3x3 or eye(3)
pandaArms.wTog = [w_g_ori w_g_pos; 0 0 0 1];

% change and check with the different goal pose of the object
% pandaArms.wTog = [eye(3), [0.5 0 0.025]'; 0 0 0 1];

%% Mission configuration

mission.prev_action = "go_to";
mission.current_action = "go_to";

mission.phase = 1;
mission.phase_time = 0;

% Define the active tasks for each phase of the mission -> objectives
% task -> objective
% T = move tool task
% JL = joint limits task
% MA = minimum altitude task
% RC = rigid constraint task

mission.actions.go_to.tasks = ["MA","JL","T"];
mission.actions.coop_manip.tasks = ["MA","JL","RC","T"];
mission.actions.end_motion.tasks = "MA";     % MA and T 

%% CONTROL LOOP
disp('STARTED THE SIMULATION');
for t = 0:deltat:end_time
    % Receive UDP packets - DO NOT EDIT
    if real_robot == true
        dataLeft = step(hudprLeft);
        dataRight = step(hudprRight);
        % wait for data (to decide)
         if t == 0
             while(isempty(dataLeft))
                 dataLeft = step(hudprLeft);
                 pause(deltat);
             end
             while(isempty(dataRight))
                 dataRight = step(hudprRight);
                 pause(deltat);
             end
         end
        qL = typecast(dataLeft, 'double');
        qR = typecast(dataRight, 'double');
        pandaArms.ArmL.q = qL;
        pandaArms.ArmR.q = qR;
    end
    
    % update all the involved variables
    pandaArms = UpdateTransforms(pandaArms, mission);
    pandaArms = ComputeJacobians(pandaArms, mission);
    pandaArms = ComputeActivationFunctions(pandaArms,mission);
    pandaArms = ComputeTaskReferences(pandaArms,mission);

    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar = zeros(14,1);
    Qp = eye(14);

    % Used by the Move-To task
    tool_jacobian_L = zeros(6, 7);
    tool_jacobian_R = zeros(6, 7);
    if (mission.phase == 1)
        % In this phase the tool frame coincide with the center of the
        % gripper
        tool_jacobian_L = pandaArms.ArmL.wJt;
        tool_jacobian_R = pandaArms.ArmR.wJt;
    elseif(mission.phase == 2)
        % In this phase the tool frame coincide with the object frame
        tool_jacobian_L = pandaArms.ArmL.wJo;
        tool_jacobian_R = pandaArms.ArmR.wJo;
    end

    % ADD minimum distance from table
    pandaArms.ArmL.minimum_altitude = pandaArms.ArmL.wTt(3,4);    % minimum altitude of the left tool from the table 
    pandaArms.ArmR.minimum_altitude = pandaArms.ArmR.wTt(3,4);    % minimum altitude of the right tool from the table 

    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    
    % Bimanual system TPIK
    % ...
    % Task: Tool Move-To
    [Qp, ydotbar] = iCAT_task(pandaArms.A.rc, pandaArms.Jokc, Qp, ydotbar, pandaArms.xdot.rc, 0.0001, 0.01, 10);  % RC
    [Qp, ydotbar] = iCAT_task(pandaArms.A.ma, pandaArms.Jma, Qp, ydotbar, [pandaArms.ArmL.xdot.alt; pandaArms.ArmR.xdot.alt], 0.0001, 0.01, 10);  % MA
    [Qp, ydotbar] = iCAT_task(pandaArms.A.jl, pandaArms.Jjl, Qp, ydotbar, [pandaArms.ArmL.xdot.jl; pandaArms.ArmR.xdot.jl], 0.0001, 0.01, 10);  % JL
    [Qp, ydotbar] = iCAT_task(pandaArms.A.tool, [tool_jacobian_L zeros(6,7); zeros(6,7) tool_jacobian_R], Qp, ydotbar, [pandaArms.ArmL.xdot.tool; pandaArms.ArmR.xdot.tool], 0.0001, 0.01, 10); % tool position and orientation
    [Qp, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 0.0001, 0.01, 10);    % this task should be the last one (a low-priority task or a "null task")
 

    % get the two variables for integration
    pandaArms.ArmL.q_dot = ydotbar(1:7);
    pandaArms.ArmR.q_dot = ydotbar(8:14);

    pandaArms.ArmL.x = tool_jacobian_L * pandaArms.ArmL.q_dot;
    pandaArms.ArmR.x = tool_jacobian_R * pandaArms.ArmR.q_dot;

    % Integration
	pandaArms.ArmL.q = pandaArms.ArmL.q(1:7) + pandaArms.ArmL.q_dot*deltat;    
    pandaArms.ArmR.q = pandaArms.ArmR.q(1:7) + pandaArms.ArmR.q_dot*deltat;

    %Send udp packets [q_dot1, ..., q_dot7] DO NOT CHANGE
    if real_robot == false
        pandaArms.ArmL.q = pandaArms.ArmL.q(1:7) + pandaArms.ArmL.q_dot*deltat; 
        pandaArms.ArmR.q = pandaArms.ArmR.q(1:7) + pandaArms.ArmR.q_dot*deltat; 
    end
    %Send udp packets [q_dot1, ..., q_dot7]
    if real_robot == true
        step(hudpsLeft,[t;pandaArms.ArmL.q_dot]);
        step(hudpsRight,[t;pandaArms.ArmR.q_dot]);
    else 
        step(hudps,[pandaArms.ArmL.q',pandaArms.ArmR.q'])
    end
    
    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat;
    [pandaArms,mission] = UpdateMissionPhase(pandaArms, mission);
   
    % Update data plot
    plt = UpdateDataPlot(plt,pandaArms,t,loop, mission);
    loop = loop + 1;
    % add debug prints here
    if (mod(t,0.1) == 0)
        disp("time: " + num2str(t));
        disp("phase: " + num2str(mission.phase));
        if (mission.phase == 1)
            %add debug prints phase 1 here
            [ang, lin] = CartError(pandaArms.ArmR.wTt, pandaArms.ArmL.wTt);
            disp("Angular Distance: " + num2str(ang'));
            disp("Linear Distance: " + num2str(lin'));
            % fprintf('problem in phase 1');

        elseif (mission.phase == 2)
            %add debug prints phase 2 here
            [ang, lin] = CartError(pandaArms.ArmR.wTt, pandaArms.ArmL.wTt);            
            disp("Angular Distance: " + num2str(ang'));
            disp("Linear Distance: " + num2str(lin'));            
            disp("ArmL z: " + num2str(pandaArms.ArmL.wTt(3,4)));
            disp("ArmR z: " + num2str(pandaArms.ArmR.wTt(3,4)));            
            disp("ArmL xdot_tool: " + num2str(pandaArms.ArmL.xdot.tool'));
            disp("ArmR xdot_tool: " + num2str(pandaArms.ArmR.xdot.tool'));            
            disp("ArmL velocity: " + num2str(pandaArms.ArmL.x'));
            disp("ArmR velocity: " + num2str(pandaArms.ArmR.x'));
            % fprintf('problem in phase 2');
        end
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % WARNING: MUST BE ENABLED IF CONTROLLING REAL ROBOT !
    % SlowdownToRealtime(deltat);
    
end

PrintPlot(plt, pandaArms);
end
