function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

if(mission.current_action == "safe_nav") 
    previous_action_list = mission.actions.safe_nav;
    current_action_list = mission.actions.safe_nav;
elseif(mission.current_action == "landing") 
    previous_action_list = mission.actions.safe_nav;
    current_action_list = mission.actions.landing;
elseif(mission.current_action == "grasping")
    previous_action_list = mission.actions.landing;
    current_action_list = mission.actions.grasping;
end


% arm tool position control
% always active
uvms.A.t = eye(6);

% grasping task
uvms.A.g = eye(6) * ActionTransition("G", previous_action_list, current_action_list, mission.phase_time);

% vehicle position and orientation tasks
uvms.A.vp = eye(3) * ActionTransition("VP", previous_action_list, current_action_list, mission.phase_time);
uvms.A.vo = eye(3) * ActionTransition("VO", previous_action_list, current_action_list, mission.phase_time);

% vehicle minimum altitude
uvms.A.ma = DecreasingBellShapedFunction(1, 2, 0, 1, uvms.altitude) * ActionTransition("MA", previous_action_list, current_action_list, mission.phase_time);

% zero vehicle altitude
uvms.A.a = 1 * ActionTransition("A", previous_action_list, current_action_list, mission.phase_time);

% horizontal attitude
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw; 
v_kv = [0 0 1]';
rho_ha = ReducedVersorLemma(v_kw,  v_kv);
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(rho_ha)) * ActionTransition("HA", previous_action_list, current_action_list, mission.phase_time);

% zero velocity 
uvms.A.zv = eye(6) * ActionTransition("ZV", previous_action_list, current_action_list, mission.phase_time);

% vehicle alignment to the target task
% w_iv = uvms.wTv(1:3, 1:3) * [1 0 0]';
% w_iv_orth = ( eye(3, 3) - w_kw * w_kw' ) * w_iv;
% uvms.w_align_target = [12.2025   37.3748  -39.8860]';
% w_dtilde = ( eye(3, 3) - w_kw * w_kw' ) * ( uvms.w_align_target - uvms.p(1:3) );
% w_ntilde = w_dtilde / norm( w_dtilde );
% uvms.w_rho_align = ReducedVersorLemma( w_ntilde, w_iv_orth );
uvms.A.al = IncreasingBellShapedFunction(0.001, 0.05, 0, 1, norm(uvms.w_rho_align))* ActionTransition("AL", previous_action_list, current_action_list, mission.phase_time);

end