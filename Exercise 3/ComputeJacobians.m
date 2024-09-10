function [pandaArm] = ComputeJacobians(pandaArm,mission)
% compute the relevant Jacobians here
% joint limits
% tool-frame position control (to do)
% initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352, -0.0305686, 1.53975, 0.753872] ) 
%
% remember: the control vector is:
% [q_dot] 
% [qdot_1, qdot_2, ..., qdot_7]
%
% therefore all task jacobians should be of dimensions
% m x 14
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]

% Jacobian from base to end-effector
pandaArm.bJe = geometricJacobian(pandaArm.franka,[pandaArm.q',0,0],'panda_link7');
pandaArm.bJe = pandaArm.bJe(:,1:7);   % <-

% THE JACOBIAN bJe has dimension 6x9 (the matlab model include the joint
% of the gripper). YOU MUST RESIZE THE MATRIX IN ORDER TO CONTROL ONLY THE
% 7 JOINTS OF THE ROBOTIC ARM. -> exclude the gripper's joints (2)
% To control only the 7 joints of the robotic arm, this matrix needs to be resized.

skew_r = skew(pandaArm.wTb(1:3,1:3) * pandaArm.bTe(1:3,1:3) * pandaArm.eTt(1:3,4));
wSt_e = [eye(3), zeros(3); skew_r', eye(3)];    % 6x6 spatial transformation matrix

pandaArm.wJt = wSt_e * [pandaArm.wTb(1:3,1:3) zeros(3); zeros(3) pandaArm.wTb(1:3,1:3)] * pandaArm.bJe;

if mission.phase == 2

    w_r_o = pandaArm.wTo(1:3,4);    % Position vector from the world frame to the object
    w_r_t = pandaArm.wTt(1:3,4);    % Position vector from the world frame to the tool
    t_r_o = w_r_o - w_r_t;          % distance between <o> and <t> 
    skew_t_r_o = skew(t_r_o); 

    wSo_t = [eye(3), zeros(3); skew_t_r_o', eye(3)]; % Spatial Transformation Matrix 

    pandaArm.wJo = wSo_t * pandaArm.wJt;
end

% joint limits 
pandaArm.Jjl = eye(7,7);

% minimum altitude 
pandaArm.Jma = pandaArm.wJt(6,:);

end
