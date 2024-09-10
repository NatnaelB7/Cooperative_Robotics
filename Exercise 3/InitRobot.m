function [pandaArm] = InitRobot(model,wTb)

%% DO NOT CHANGE FROM HERE ...
% Init two field of the main structure pandaArm containing the two robot
% model
pandaArm = model;
% Init robot basic informations (q_init, transformation matrices ...)
pandaArm.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
pandaArm.q_dot = [0 0 0 0 0 0 0]';
pandaArm.alt = 0.20;

pandaArm.bTe = getTransform(pandaArm.franka,[pandaArm.q',0,0],'panda_link7');
pandaArm.wTb = wTb;
pandaArm.wTe = pandaArm.wTb*pandaArm.bTe;

% joint limits corresponding to the actual Panda by Franka arm configuration
pandaArm.jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
pandaArm.jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

% Init relevance Jacobians
pandaArm.bJe = eye(6,7);
pandaArm.Jjl = [];

%% ... TO HERE
% Init Task Reference vectors
pandaArm.xdot.tool = [];
pandaArm.xdot.alt = [];
pandaArm.xdot.rc = zeros(6,1);

pandaArm.xdot.non_coop = zeros(6,1);
pandaArm.xdot.coop = zeros(6,1);

% Init Activation function for activate or deactivate tasks
pandaArm.A.tool = zeros(6);
pandaArm.A.jl = zeros(7);
pandaArm.A.rc = zeros(6);
pandaArm.A.coop = zeros(6);
pandaArm.A.ma = 0;

% distance between tool and object
pandaArm.t_r_o = zeros(3,1);

% linear and angular cartesian positions
pandaArm.p = zeros(6,1);

pandaArm.gain = 0;

end

