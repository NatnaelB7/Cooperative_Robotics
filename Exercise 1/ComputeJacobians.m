function [uvms] = ComputeJacobians(uvms)

% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
% with the vehicle velocities projected on <v>
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%
% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

% vehicle distance and orientation tasks
uvms.Jvp = [zeros(3,7) -eye(3) zeros(3)];   % [zeros(3,7) uvms.wTv(1:3,1:3) zeros(3)];
uvms.Jvo = [zeros(3,7), zeros(3) eye(3)];   % [zeros(3,7) zeros(3) uvms.wTv(1:3,1:3)];

% zero altitude and minimun altitude tasks
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw; 
uvms.Jma = [zeros(1,7), v_kw', zeros(1,3)];
uvms.Ja = [zeros(1,7), v_kw', zeros(1,3)];

% horizontal attitude
v_kv = [0 0 1]';
rho_ha = ReducedVersorLemma(v_kw,  v_kv);
if (norm(rho_ha) > 0) 
    n_ha = rho_ha / norm(rho_ha);
    uvms.Jha = [zeros(1,7) zeros(1,3) n_ha'];
else
    uvms.Jha = [zeros(1,13)];
end

% zero velocity constraint task
uvms.Jzv = [ zeros(6, 7), [uvms.wTv(1:3, 1:3); zeros(3)], [zeros(3); uvms.wTv(1:3, 1:3)] ];

% grasping task

% Vehicle alignment to the target task
w_iv = uvms.wTv(1:3, 1:3) * [1 0 0]';
w_iv_orth = ( eye(3, 3) - w_kw * w_kw' ) * w_iv;
uvms.w_align_target = [12.2025   37.3748  -39.8860]';
w_dtilde = ( eye(3, 3) - w_kw * w_kw' ) * ( uvms.w_align_target - uvms.p(1:3) );
w_ntilde = w_dtilde / norm( w_dtilde );
uvms.w_rho_align = ReducedVersorLemma( w_ntilde, w_iv_orth );
th_tg_align = norm( uvms.w_rho_align );
w_v_align = ( skew(w_ntilde)*w_iv_orth ) / sin( th_tg_align );
uvms.Jal = [ zeros(1, 7), zeros(1, 3), w_v_align'*uvms.wTv(1:3, 1:3) ];

end