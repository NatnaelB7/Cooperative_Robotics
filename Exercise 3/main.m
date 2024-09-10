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
%% TO HERE

% Init robot model
wTb_left = eye(4);      %fixed transformation word -> base left
right_arm_base_position = [1.06 -0.01 0]';
wTb_right = [rotation(0,0,pi) right_arm_base_position; 0 0 0 1]; %fixed transformation word -> base right

pandaArm1 = InitRobot(model,wTb_left);
pandaArm2 = InitRobot(model,wTb_right);

% Preallocation
plt = InitDataPlot(maxloops);

% Init object frame
obj_length = 0.06;
w_obj_pos = [0.5 0 0.3]';
w_obj_ori = rotation(0,0,0);  % eye(3)
pandaArm1.wTo = [w_obj_ori w_obj_pos; 0 0 0 1];
pandaArm2.wTo = pandaArm1.wTo;

theta = deg2rad(-44.9949);      % FIXED ANGLE BETWEEN EE AND TOOL 
tool_length = 0.2104;           % FIXED DISTANCE BETWEEN EE AND TOOL

% Define transformation matrix from ee to tool.
e_tool_pos = [0 0 tool_length]';
e_tool_ori = rotation(0,0,theta);
pandaArm1.eTt = [e_tool_ori e_tool_pos; 0 0 0 1];
pandaArm2.eTt = pandaArm1.eTt;

% Transformation matrix from <t> to <w>
pandaArm1.wTt = pandaArm1.wTe * pandaArm1.eTt;
pandaArm2.wTt = pandaArm2.wTe * pandaArm2.eTt;

%% Defines the goal position for the end-effector/tool position task

% First goal reach the grasping points (Reach the object)
theta_2 = deg2rad(20);
w_O_grasp_pos_left = w_obj_pos - [obj_length/2 0 0]';
w_O_grasp_pos_right = w_obj_pos + [obj_length/2 0 0]';
pandaArm1.wTg = [pandaArm1.wTt(1:3,1:3) * rotation(0,theta_2,0), w_O_grasp_pos_left; 0 0 0 1];
pandaArm2.wTg = [pandaArm2.wTt(1:3,1:3) * rotation(0,theta_2,0), w_O_grasp_pos_right; 0 0 0 1];

% Second goal move the object
w_g_pos = [0.60 0.40 0.48]';
w_g_ori = rotation(0,0,0);  % identity matrix 3x3 or eye(3)
pandaArm1.wTog = [w_g_ori w_g_pos; 0 0 0 1];
pandaArm2.wTog = [w_g_ori w_g_pos; 0 0 0 1];

% With different goal 
% pandaArm1.wTog = [w_g_ori [0.65 -0.35 0.28]'; 0 0 0 1];
% pandaArm2.wTog = pandaArm1.wTog;

%% Mission configuration

mission.prev_action = "go_to";
mission.current_action = "go_to";

mission.phase = 1;
mission.phase_time = 0;

% Define the active tasks for each phase of the mission
% T = move tool task   --- move to the grasping point
% JL = joint limits task
% MA = minimum altitude task
% RC = rigid constraint task  ---- RC is replaced by MM 
% MM = mutual motion task

mission.actions.go_to.tasks = ["MA","JL","T"];
mission.actions.coop_manip.tasks = ["MM","MA","JL","T"];   
mission.actions.end_motion.tasks = "MA";     % MA and T 

%% CONTROL LOOP
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
        pandaArm1.q = qL;
        pandaArm2.q = qR;
    end

    pandaArm1.gain = 2.5;     % 
    pandaArm2.gain = 0.5;     %

    % update all the involved variables
    [pandaArm1] = UpdateTransforms(pandaArm1,mission);
    [pandaArm2] = UpdateTransforms(pandaArm2,mission);
    [pandaArm1] = ComputeJacobians(pandaArm1,mission);
    [pandaArm2] = ComputeJacobians(pandaArm2,mission);
    [pandaArm1] = ComputeActivationFunctions(pandaArm1,mission);
    [pandaArm2] = ComputeActivationFunctions(pandaArm2,mission);
    [pandaArm1] = ComputeTaskReferences(pandaArm1,mission);
    [pandaArm2] = ComputeTaskReferences(pandaArm2,mission);


    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar1 = zeros(7,1);
    Qp1 = eye(7);
    ydotbar2 = zeros(7,1);
    Qp2 = eye(7);

    % Used by the Move-To task
    tool_jacobian_L = zeros(6, 7);
    tool_jacobian_R = zeros(6, 7);
    if (mission.phase == 1)
        % In this phase the tool frame coincide with the center of the
        % gripper
        tool_jacobian_L = pandaArm1.wJt;
        tool_jacobian_R = pandaArm2.wJt;
    elseif(mission.phase == 2)
        % In this phase the tool frame coincide with the object frame
        tool_jacobian_L = pandaArm1.wJo;
        tool_jacobian_R = pandaArm2.wJo;
    end
    
    
    % ADD minimum distance from table
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    
    ydotbar1_init = ydotbar1;
    ydotbar2_init = ydotbar2;

    %%%%%%%%%%%%% Non Cooperative Interaction %%%%%%%%%%%%%%

    % First Manipulator TPIK (left -> Arm1)
    % Task: Tool Move-To
    [Qp1, ydotbar1] = iCAT_task(pandaArm1.A.ma, pandaArm1.Jma, Qp1, ydotbar1, pandaArm1.xdot.alt, 0.0001, 0.01, 10);  % MA
    [Qp1, ydotbar1] = iCAT_task(pandaArm1.A.jl, pandaArm1.Jjl, Qp1, ydotbar1, pandaArm1.xdot.jl, 0.0001, 0.01, 10);   % JL
    [Qp1, ydotbar1] = iCAT_task(pandaArm1.A.tool, tool_jacobian_L, Qp1, ydotbar1, pandaArm1.xdot.tool, 0.0001, 0.01, 10); % T
    [Qp1, ydotbar1] = iCAT_task(eye(7), eye(7), Qp1, ydotbar1, zeros(7,1), 0.0001,   0.01, 10);    % this task should be the last one

    % Second manipulator TPIK (right -> Arm2)
    % Task: Tool Move-To
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.ma, pandaArm2.Jma, Qp2, ydotbar2, pandaArm2.xdot.alt, 0.0001, 0.01, 10);  % MA
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.jl, pandaArm2.Jjl, Qp2, ydotbar2, pandaArm2.xdot.jl, 0.0001, 0.01, 10);   % JL
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.tool, tool_jacobian_R, Qp2, ydotbar2, pandaArm2.xdot.tool, 0.0001, 0.01, 10); % T
    [Qp2, ydotbar2] = iCAT_task(eye(7), eye(7), Qp2, ydotbar2, zeros(7,1), 0.0001,   0.01, 10);    % this task should be the last one
    
    % COOPERATION hierarchy
    % SAVE THE NON COOPERATIVE VELOCITIES COMPUTED

    non_coop_xdot1 = tool_jacobian_L * ydotbar1;       % xdot = J * ydotbar 
    non_coop_xdot2 = tool_jacobian_R * ydotbar2;

    % Then, save the non-cooperative tool velocities    
    pandaArm1.xdot.non_coop = non_coop_xdot1;     
    pandaArm2.xdot.non_coop = non_coop_xdot2;

    if (mission.phase == 2)        
        Qp1 = eye(7);
        Qp2 = eye(7);

        % sending H1, H2, xdot1_non_coop and xdot2_non_coop         
        H1 = pandaArm1.wJo*pinv(pandaArm1.wJo);      % Hi = J * Jinv
        H2 = pandaArm2.wJo*pinv(pandaArm2.wJo);    
        C = [H1 -H2];    

        % compute weights (mu_0, mu_a and mu_b)        
        mu_0 = 0.01;
        mu_a = mu_0 + norm(pandaArm1.xdot.tool - pandaArm1.xdot.non_coop);  % desired velocity - non cooperative velocity        
        mu_b = mu_0 + norm(pandaArm2.xdot.tool - pandaArm2.xdot.non_coop);

        disp("mu_a: " + num2str(mu_a));
        disp("mu_b: " + num2str(mu_b));

        % compute the cooperative velocities        
        coop_xdot1 = (1 / (mu_a + mu_b)) * (mu_a * non_coop_xdot1 + mu_b * non_coop_xdot2);
        coop_xdot2 = coop_xdot1;

        % store cooperative tool velocities        
        pandaArm1.xdot.coop = coop_xdot1;
        pandaArm2.xdot.coop = coop_xdot2;  

        % feasible cooperative velocities        
        feasible_coop_xdot = [H1 zeros(6); zeros(6) H2] * (eye(12) - pinv(C)*C) * [coop_xdot1; coop_xdot2];
        a = zeros(7,1);       %   
        b = zeros(7,1);       %


    %%%%%%%%%%%%%%%%%%%%% Cooperative Interaction %%%%%%%%%%%%%%%%%%%%%

    % Task: Arm1 (Left Arm) Cooperation
    [Qp1, ydotbar1] = iCAT_task(pandaArm1.A.coop, tool_jacobian_L, Qp1, ydotbar1_init, feasible_coop_xdot(1:6), 0.0001, 0.01, 10);  % MM
    [Qp1, ydotbar1] = iCAT_task(pandaArm1.A.ma, pandaArm1.Jma, Qp1, ydotbar1, pandaArm1.xdot.alt, 0.0001, 0.01, 10);  % MA
    [Qp1, ydotbar1] = iCAT_task(pandaArm1.A.jl, pandaArm1.Jjl, Qp1, ydotbar1, pandaArm1.xdot.jl, 0.0001, 0.01, 10);   % JL
    [Qp1, ydotbar1] = iCAT_task(pandaArm1.A.tool, tool_jacobian_L, Qp1, ydotbar1, pandaArm1.xdot.tool, 0.0001, 0.01, 10);   % T
    [Qp1, ydotbar1] = iCAT_task(eye(7), eye(7), Qp1, ydotbar1, zeros(7,1), 0.0001,   0.01, 10);   % this task should be the last one

    % Task: Arm2 (Right Arm) Cooperation
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.coop, tool_jacobian_R, Qp2, ydotbar2_init, feasible_coop_xdot(7:12), 0.0001, 0.01, 10); % MM
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.ma, pandaArm2.Jma, Qp2, ydotbar2, pandaArm2.xdot.alt, 0.0001, 0.01, 10);  % MA
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.jl, pandaArm2.Jjl, Qp2, ydotbar2, pandaArm2.xdot.jl, 0.0001, 0.01, 10);   % JL
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.tool, tool_jacobian_R, Qp2, ydotbar2, pandaArm2.xdot.tool, 0.0001, 0.01, 10);   % T
    [Qp2, ydotbar2] = iCAT_task(eye(7), eye(7), Qp2, ydotbar2, zeros(7,1), 0.0001,   0.01, 10);  % this task should be the last one  
    
    end 

    % get the two variables for integration
    pandaArm1.q_dot = ydotbar1(1:7);
    pandaArm2.q_dot = ydotbar2(1:7);

    pandaArm1.x = tool_jacobian_L * pandaArm1.q_dot;
    pandaArm2.x = tool_jacobian_R * pandaArm2.q_dot;

    % Integration
	pandaArm1.q = pandaArm1.q(1:7) + pandaArm1.q_dot*deltat;    
    pandaArm2.q = pandaArm2.q(1:7) + pandaArm2.q_dot*deltat;  

    pandaArm1.p = pandaArm1.p + pandaArm1.x*deltat;
    pandaArm2.p = pandaArm2.p + pandaArm2.x*deltat;

    % Send udp packets [q_dot1, ..., q_dot7] DO NOT CHANGE
    if real_robot == false
        pandaArm1.q = pandaArm1.q(1:7) + pandaArm1.q_dot*deltat; 
        pandaArm2.q = pandaArm2.q(1:7) + pandaArm2.q_dot*deltat; 
    end
    % Send udp packets [q_dot1, ..., q_dot7]
    if real_robot == true
        step(hudpsLeft,[t;pandaArm1.q_dot]);
        step(hudpsRight,[t;pandaArm2.q_dot]);
    else 
        step(hudps,[pandaArm1.q',pandaArm2.q'])
    end

    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat;
    [pandaArm1,pandaArm2,mission] = UpdateMissionPhase(pandaArm1,pandaArm2,mission);

    % Compute distance between tools for plotting
    pandaArm1.dist_tools = norm(pandaArm1.wTt(1:3, 4) - pandaArm2.wTt(1:3, 4));

    % Update data for plots
    plt = UpdateDataPlot(plt,pandaArm1,pandaArm2,t,loop, mission);

    loop = loop + 1;
    % add debug prints here
    if (mod(t,0.1) == 0)
        disp("time: " + num2str(t));
        disp("phase: " + num2str(mission.phase));
       if (mission.phase == 1)
            %add debug prints phase 1 here
            [ang1, lin1] = CartError(pandaArm1.wTg, pandaArm1.wTt);
            [ang2, lin2] = CartError(pandaArm2.wTg, pandaArm2.wTt);
            disp("Error of Arm1: " + sprintf( "%f ", ang1) + sprintf("%f ",lin1));
            disp("Error of Arm2: " + sprintf( "%f ", ang2) + sprintf("%f ",lin2));
            % fprintf('problem in phase 1');

       elseif (mission.phase == 2)
           %add debug prints phase 2 here
           [ang1, lin1] = CartError(pandaArm1.wTog, pandaArm1.wTo);
           [ang2, lin2] = CartError(pandaArm2.wTog, pandaArm2.wTo);
           disp("Error of Arm1: " + sprintf( "%f ", ang1) + sprintf("%f ",lin1));
           disp("Error of Arm2: " + sprintf( "%f ", ang2) + sprintf("%f ",lin2));
           % fprintf('problem in phase 1');
       end 
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % WARNING: MUST BE ENABLED IF CONTROLLING REAL ROBOT !
    % SlowdownToRealtime(deltat);
    
end
PrintPlot(plt);

end
