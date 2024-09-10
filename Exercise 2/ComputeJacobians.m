function [pandaArm] = ComputeJacobians(pandaArm,mission)
% compute the relevant Jacobians here
% joint limits
% tool-frame position control (to do)
% initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352, 
% -0.0305686, 1.53975, 0.753872] ) 
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

% Left Arm base to ee Jacobian
pandaArm.ArmL.bJe = geometricJacobian(pandaArm.ArmL.franka, ...
    [pandaArm.ArmL.q',0,0],'panda_link7');%DO NOT EDIT
pandaArm.ArmL.bJe = pandaArm.ArmL.bJe(:,1:7);   % <-

% Right Arm base to ee Jacobian
pandaArm.ArmR.bJe = geometricJacobian(pandaArm.ArmR.franka, ...
    [pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT
pandaArm.ArmR.bJe = pandaArm.ArmR.bJe (:,1:7);   % <-

% Top three rows are angular velocities, bottom three linear velocities
skew_pos_left = skew(pandaArm.ArmL.wTe(1:3,1:3) * pandaArm.ArmL.eTt(1:3,4));
wSt_e_left = [eye(3), zeros(3); skew_pos_left', eye(3)];

skew_pos_right = skew(pandaArm.ArmR.wTb(1:3,1:3) * pandaArm.ArmR.bTe(1:3,1:3) * pandaArm.ArmR.eTt(1:3,4));
wSt_e_right = [eye(3), zeros(3); skew_pos_right', eye(3)];

pandaArm.ArmL.wJt  = wSt_e_left * pandaArm.ArmL.bJe;
pandaArm.ArmR.wJt  = wSt_e_right * ([pandaArm.ArmR.wTb(1:3,1:3) zeros(3); zeros(3) pandaArm.ArmR.wTb(1:3,1:3)] * pandaArm.ArmR.bJe);

if (mission.phase == 2)  
     
    w_pos_o_left = pandaArm.ArmL.wTo(1:3,4); 
    w_pos_t_left = pandaArm.ArmL.wTt(1:3,4); 
    t_pos_o_left = w_pos_o_left - w_pos_t_left;   % distance between <o> and <t> 
    skew_t_pos_o_left = skew(t_pos_o_left); 
 
    wSo_t_left = [eye(3), zeros(3); skew_t_pos_o_left', eye(3)]; 
 
    w_pos_o_right = pandaArm.ArmR.wTo(1:3,4); 
    w_pos_t_right = pandaArm.ArmR.wTt(1:3,4); 
    t_pos_o_right = w_pos_o_right - w_pos_t_right; 
    skew_t_pos_o_right= skew(t_pos_o_right); 
 
    wSo_t_right= [eye(3), zeros(3); skew_t_pos_o_right', eye(3)]; 
 
    pandaArm.ArmL.wJo = wSo_t_left * pandaArm.ArmL.wJt; 
    pandaArm.ArmR.wJo = wSo_t_right * pandaArm.ArmR.wJt; 
 
    pandaArm.Jokc = [pandaArm.ArmL.wJo -pandaArm.ArmR.wJo]; 
end 
% Common Jacobians 
 
% joint limits 
pandaArm.Jjl = [eye(7,7) zeros(7,7); zeros(7,7) eye(7,7)]; % why xdot is 7x1 (14x1) 
 
% minimum altitude 
pandaArm.Jma = [pandaArm.ArmL.wJt(6,:) zeros(1,7); zeros(1,7) pandaArm.ArmR.wJt(6,:)]; 
 
end