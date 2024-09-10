function [ plt ] = UpdateDataPlot( plt, pandaArm1, pandaArm2, t, loop, mission )

% this function samples the variables contained in the structure pandaArm
% and saves them in arrays inside the struct plt
% this allows to have the time history of the datauvms for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(:, loop) = t;
plt.q(:, loop) = pandaArm1.q;
plt.q_dot(:, loop) = pandaArm1.q_dot;
plt.q2(:, loop) = pandaArm2.q;
plt.q_dot2(:, loop) = pandaArm2.q_dot;

%% Activation Functions 

% Arm1 Activation Functions 
plt.arm1.a.jl(:,loop) = diag(pandaArm1.A.jl);
plt.arm1.a.ma(:,loop) = pandaArm1.A.ma;
plt.arm1.a.tool(:,loop) = diag(pandaArm1.A.tool);
plt.arm1.a.coop(:,loop) = diag(pandaArm1.A.coop);
% plt.arm1.a.rc(:,loop) = diag(pandaArm1.A.rc);

% Arm2 Activation Functions 
plt.arm2.a.jl(:,loop) = diag(pandaArm2.A.jl);
plt.arm2.a.ma(:,loop) = pandaArm2.A.ma;
plt.arm2.a.tool(:,loop) = diag(pandaArm2.A.tool);
plt.arm2.a.coop(:,loop) = diag(pandaArm2.A.coop);
% plt.arm2.a.rc(:,loop) = diag(pandaArm2.A.rc);

%% Plot: Desired object velocity

% Desired Velocities
plt.xdot1_tool(:, loop) = pandaArm1.xdot.tool;
plt.xdot2_tool(:, loop) = pandaArm2.xdot.tool;

% Non-cooperative Desired Velocity
plt.xdot1_non_coop(:, loop) = pandaArm1.xdot.non_coop;
plt.xdot2_non_coop(:, loop) = pandaArm2.xdot.non_coop;

% Cooperative Desired Velocity 
if (mission.phase == 1)
    plt.xdot1_coop(:, loop) = zeros(6,1);
    plt.xdot2_coop(:, loop) = zeros(6,1);   % xdot2_coop
else
    plt.xdot1_coop(:, loop) = pandaArm1.xdot.coop;
    plt.xdot2_coop(:, loop) = pandaArm2.xdot.coop;
end 

%% End effector velocities 
% Left Arm -> Arm1
plt.xdot1(:, loop) = pandaArm1.x;

% Right Arm -> Arm2
plt.xdot2(:, loop) = pandaArm2.x;

%% Cartesian Positions
plt.arm1.p(:, loop) = pandaArm1.p;
plt.arm2.p(:, loop) = pandaArm2.p;

mu_0 = 0.01;
plt.arm1.m(:,loop) = mu_0 + norm(pandaArm1.xdot.tool - pandaArm1.xdot.non_coop);  % desired non-cooperative velocity
plt.arm2.m(:,loop) = mu_0 + norm(pandaArm2.xdot.tool - pandaArm2.xdot.non_coop);

%% mission 
plt.phase(loop) = mission.phase;

% Plot: manipulability task activation function

end