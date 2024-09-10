function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
% Theoretical Part: xdotbar = λ(xbar − x), λ > 0

% Compute distance between tools for plotting
pandaArm.dist_tools = norm(pandaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmR.wTt(1:3, 4));

% gains which are tuned experimentally
gain = 0.5;
gainL = 1.5;   % Increases responsiveness and speeds up convergence to the desired state
gainR = 0.5;

% Compute minimum altitude reference ALWAYS
minimum_alt = 0.15;    % 0.15 + 0.05(delta) = 0.20

pandaArm.ArmL.xdot.alt = gainL * (minimum_alt - pandaArm.ArmL.minimum_altitude);    % minimum_altitude
pandaArm.ArmR.xdot.alt = gainR * (minimum_alt - pandaArm.ArmR.minimum_altitude);    % xbardot = λ(xbar − x), λ > 0

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin

pandaArm.ArmL.xdot.jl = gainL * (((pandaArm.jlmax + pandaArm.jlmin) / 2) - pandaArm.ArmL.q);
pandaArm.ArmR.xdot.jl = gainR * (((pandaArm.jlmax + pandaArm.jlmin) / 2) - pandaArm.ArmR.q);

switch mission.phase
    case 1
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [angL, linL] = CartError(pandaArm.ArmL.wTg, pandaArm.ArmL.wTt);

        ang_error = gainL * angL;
        lin_error = gainL * linL;
       
        pandaArm.ArmL.xdot.tool = [ang_error; lin_error];
        % limit the requested velocities...
        pandaArm.ArmL.xdot.tool(1:3) = Saturate(pandaArm.ArmL.xdot.tool(1:3), deg2rad(180));  % 180 deg/s (max allowed angular velocity, taken from the datasheet)
        pandaArm.ArmL.xdot.tool(4:6) = Saturate(pandaArm.ArmL.xdot.tool(4:6), 2);    % 2 m/s (max allowed linear velocity, taken from the datasheet)

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [angR, linR] = CartError(pandaArm.ArmR.wTg, pandaArm.ArmR.wTt);

        ang_error = gainR * angR;
        lin_error = gainR * linR;
       
        pandaArm.ArmR.xdot.tool = [ang_error; lin_error];
        % limit the requested velocities...velocities are saturated to prevent excessive speeds
        pandaArm.ArmR.xdot.tool(1:3) = Saturate(pandaArm.ArmR.xdot.tool(1:3), deg2rad(150));
        pandaArm.ArmR.xdot.tool(4:6) = Saturate(pandaArm.ArmR.xdot.tool(4:6), 2);

    case 2
        % Perform the rigid grasp of the object and move it

        % COMMON
        % -----------------------------------------------------------------
        % Rigid Grasp Constraint
        pandaArm.xdot.rc = zeros(6,1);

        % LEFT ARM
        % -----------------------------------------------------------------        
        % Object position and orientation task reference
        [angL, linL] = CartError(pandaArm.wTog, pandaArm.ArmL.wTo);

        ang_error = gainL * angL;
        lin_error = gainL * linL;

        pandaArm.ArmL.xdot.tool = [ang_error; lin_error];
        % limit the requested velocities...
        pandaArm.ArmL.xdot.tool(1:3) = Saturate(pandaArm.ArmL.xdot.tool(1:3), deg2rad(150));
        pandaArm.ArmL.xdot.tool(4:6) = Saturate(pandaArm.ArmL.xdot.tool(4:6), 2);

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Object position and orientation task reference
        [angR, linR] = CartError(pandaArm.wTog, pandaArm.ArmR.wTo);

        ang_error = gainR * angR;
        lin_error = gainR * linR;

        pandaArm.ArmR.xdot.tool = [ang_error; lin_error];
        % limit the requested velocities...
        pandaArm.ArmR.xdot.tool(1:3) = Saturate(pandaArm.ArmR.xdot.tool(1:3), deg2rad(150));
        pandaArm.ArmR.xdot.tool(4:6) = Saturate(pandaArm.ArmR.xdot.tool(4:6), 2);
    case 3
        % Stop any motions
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.ArmL.xdot.tool(1:3) = zeros(3,1);
        pandaArm.ArmL.xdot.tool(4:6) = zeros(3,1);

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.ArmR.xdot.tool(1:3) = zeros(3,1);
        pandaArm.ArmR.xdot.tool(4:6) = zeros(3,1);
end


