function [pandaArm] = UpdateTransforms(pandaArm, mission)
% the function updates all the transformations

% Left arm transformations
pandaArm.ArmL.bTe = getTransform(pandaArm.ArmL.franka, ...
    [pandaArm.ArmL.q',0,0],'panda_link7');%DO NOT EDIT

% Right arm transformations
pandaArm.ArmR.bTe = getTransform(pandaArm.ArmR.franka, ...
    [pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT

% <e> to <w>
pandaArm.ArmL.wTe = pandaArm.ArmL.wTb * pandaArm.ArmL.bTe;
pandaArm.ArmR.wTe = pandaArm.ArmR.wTb * pandaArm.ArmR.bTe;

% Transformation matrix from <t> to <w>
pandaArm.ArmL.wTt = pandaArm.ArmL.wTe * pandaArm.ArmL.eTt;
pandaArm.ArmR.wTt = pandaArm.ArmR.wTe * pandaArm.ArmR.eTt;

if(mission.phase == 1)
    w_rot_t_armL = pandaArm.ArmL.wTt(1:3,1:3);    % rotation
    w_pos_t_armL = pandaArm.ArmL.wTt(1:3,4);      % position
    pandaArm.ArmL.tTo = [w_rot_t_armL' -w_rot_t_armL'*w_pos_t_armL; 0 0 0 1] * pandaArm.ArmL.wTo;

    w_rot_t_armR = pandaArm.ArmR.wTt(1:3,1:3);
    w_pos_t_armR = pandaArm.ArmR.wTt(1:3,4);    
    pandaArm.ArmR.tTo = [w_rot_t_armR' -w_rot_t_armR'*w_pos_t_armR; 0 0 0 1] * pandaArm.ArmR.wTo;
end

% <o> to <w> : ASSUME <t> = <g> during entire cooperation phase
if (mission.phase == 2)
    pandaArm.ArmL.wTo = pandaArm.ArmL.wTt * pandaArm.ArmL.tTo; 
    pandaArm.ArmR.wTo = pandaArm.ArmR.wTt * pandaArm.ArmR.tTo;
    
end
