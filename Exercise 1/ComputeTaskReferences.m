function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

% vehicle position
[v_rho, v_d] = CartError(uvms.vTw*uvms.wTgv , eye(4));
uvms.xdot.vp = 0.2 * (zeros(3,1) - v_d);
uvms.xdot.vp = Saturate(uvms.xdot.vp, 1);

% vehicle orientation
uvms.xdot.vo = 0.2 * v_rho; 
% uvms.xdot.vo = Saturate(uvms.xdot.vp, 1);

% minimum altitude
uvms.xdot.ma = 0.4 * (2 - uvms.altitude);

% zero altitude
uvms.xdot.a = 0.4 * (0 - uvms.altitude);    % the vehicle should be placed on the floor

% horizontal attitude
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw; 
v_kv = [0 0 1]';
rho_ha = ReducedVersorLemma(v_kw,  v_kv);
uvms.xdot.ha = 0.3 * (0.1 - norm(rho_ha));

% zero velocity constraint
uvms.xdot.zv = 0.5 * ( zeros(6, 1) - uvms.p_dot );

% vehicle alignment to the target task
uvms.xdot.al = 0.5 * (0 - norm(uvms.w_rho_align));   % 0.5

% grasping task
rock_center = [12.2025   37.3748  -39.8860]';
rock_center2 = [eye(3) rock_center; 0 0 0 1];
[angular_error, linear_error] = CartError(rock_center2, uvms.wTv*uvms.vTt);
uvms.xdot.g = 0.4 * [zeros(3,1); linear_error];  
uvms.xdot.g(1:3) = Saturate(uvms.xdot.g(1:3), 0.2);
uvms.xdot.g(4:6) = Saturate(uvms.xdot.g(4:6), 0.2);

end