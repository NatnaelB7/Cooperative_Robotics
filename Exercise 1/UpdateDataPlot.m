function [ plt ] = UpdateDataPlot( plt, uvms, t, loop )

% this function samples the variables contained in the structure uvms
% and saves them in arrays inside the struct plt
% this allows to have the time history of the data for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(loop) = t;
plt.toolPos(:, loop) = uvms.wTt(1:3,4);
plt.q(:, loop) = uvms.q;
plt.q_dot(:, loop) = uvms.q_dot;
plt.p(:, loop) = uvms.p;
plt.p_dot(:, loop) = uvms.p_dot;


% ========Activation Functions============
% minimum altitude task
plt.A.ma(:, loop) = uvms.A.ma;
% horizontal attitude task
plt.A.ha(:, loop) = uvms.A.ha; 
% zero altitude task
plt.A.a(:, loop) = uvms.A.a;
% vehicle position control task
plt.A.vp(:, loop) = [ uvms.A.vp(1, 1) uvms.A.vp(2, 2) uvms.A.vp(3, 3) ]';
% vehicle orientation control task
plt.A.vo(:, loop) = [ uvms.A.vo(1, 1) uvms.A.vo(2, 2) uvms.A.vo(3, 3) ]';
% manipulation tool task
plt.A.t(:, loop) = [ uvms.A.t(1, 1) uvms.A.t(2, 2) uvms.A.t(3, 3) ...
                    uvms.A.t(4, 4) uvms.A.t(5, 5) uvms.A.t(6, 6) ]';

% zero velocity constraint task
plt.A.zv(:, loop) = uvms.A.zv(1, 1);
% vehicle alignment to a target task
plt.A.al(:, loop) = uvms.A.al;


% Vehicle position and orientation
plt.vpos(:, loop) = uvms.p( 1:3 );
plt.vorient(:, loop) = uvms.p( 4:6 );

% Desired vehicle position and orientation
plt.dot_vpos(:, loop) = uvms.p_dot( 1:3 );
plt.dot_vorient(:, loop) = uvms.p_dot( 4:6 );



%plt.xdot_jl(:, loop) = uvms.xdot.jl;
%plt.xdot_mu(:, loop) = uvms.xdot.mu;

% plt.xdot_t(:, loop) =  blkdiag(uvms.wTv(1:3,1:3), uvms.wTv(1:3,1:3))*uvms.xdot.t;
% plt.a(1:7, loop) = diag(uvms.A.jl);
% plt.a(8, loop) = uvms.A.mu;
% plt.a(9, loop) = uvms.A.ha(1,1);
% plt.toolx(:,loop) = uvms.wTt(1,4);
% plt.tooly(:,loop) = uvms.wTt(2,4);

end