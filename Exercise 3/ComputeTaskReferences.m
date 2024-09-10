function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
% Theoretical Part: xdotbar = λ(xbar − x), λ > 0

gain = 0.2;   % tuned experimentally

% Compute minimum altitude reference ALWAYS
minimum_alt = 0.15;    % 0.15 + 0.05(delta) = 0.20

pandaArm.xdot.alt = gain * (minimum_alt - pandaArm.wTt(3,4));

% take the smallest value, that is what matters most
% pandaArm.min_alt = min(alt_L, alt_R);

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
pandaArm.xdot.jl = gain * (((pandaArm.jlmax + pandaArm.jlmin) / 2) - pandaArm.q);

switch mission.phase
    case 1
        % Tool position and orientation task reference
        [ang, lin] = CartError(pandaArm.wTg, pandaArm.wTt);

        pandaArm.xdot.tool = pandaArm.gain * [ang; lin];
        % Limits request velocities
        pandaArm.xdot.tool(1:3) = Saturate(pandaArm.xdot.tool(1:3), deg2rad(150));
        pandaArm.xdot.tool(4:6) = Saturate(pandaArm.xdot.tool(4:6), 2);

    case 2
        % Rigid Grasp Constraint
        pandaArm.xdot.rc = zeros(6,1);
        % Object position and orientation task reference
        [ang, lin] = CartError(pandaArm.wTog, pandaArm.wTo);
       
        pandaArm.xdot.tool = pandaArm.gain * [ang; lin];
        % Limits request velocities
        pandaArm.xdot.tool(1:3) = Saturate(pandaArm.xdot.tool(1:3), deg2rad(150));
        pandaArm.xdot.tool(4:6) = Saturate(pandaArm.xdot.tool(4:6), 2);

    case 3
        % Stop any motions
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.xdot.tool(1:3) = zeros(3,1);
        pandaArm.xdot.tool(4:6) = zeros(3,1);
end