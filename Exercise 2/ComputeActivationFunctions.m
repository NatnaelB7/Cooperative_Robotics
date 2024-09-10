function [pandaArm] = ComputeActivationFunctions(pandaArm, mission)

if(mission.current_action == "go_to")
    previous_action_list = mission.actions.go_to.tasks;
    current_action_list = mission.actions.go_to.tasks;
 elseif(mission.current_action == "coop_manip")
     previous_action_list = mission.actions.go_to.tasks;
     current_action_list = mission.actions.coop_manip.tasks;
 elseif(mission.current_action == "end_motion")
     previous_action_list = mission.actions.coop_manip.tasks;
     current_action_list = mission.actions.end_motion.tasks;
end

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
         pandaArm.A.tool = eye(12) .* ActionTransition("T", previous_action_list, current_action_list, mission.phase_time);
    case 2 % Move the object holding it firmly
        % Rigid Grasp Constraint
        pandaArm.A.rc = eye(6) .* ActionTransition("RC", previous_action_list, current_action_list, mission.phase_time);
        % Move-To
        pandaArm.A.tool = eye(12) .* ActionTransition("T", previous_action_list, current_action_list, mission.phase_time);
    case 3 % STOP any motion 
        pandaArm.A.rc = eye(6) .* ActionTransition("RC", previous_action_list, current_action_list, mission.phase_time);
        
        pandaArm.A.tool = eye(12) .* ActionTransition("T", previous_action_list, current_action_list, mission.phase_time);

        
end

% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
pandaArm.A.ma_left = DecreasingBellShapedFunction(0.15, 0.20, 0, 1, pandaArm.ArmL.minimum_altitude);
pandaArm.A.ma_right = DecreasingBellShapedFunction(0.15, 0.20, 0, 1, pandaArm.ArmR.minimum_altitude);

pandaArm.A.ma = [pandaArm.A.ma_left 0;
                0         pandaArm.A.ma_right] .* ActionTransition("MA", previous_action_list, current_action_list, mission.phase_time);

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error

n = size(pandaArm.ArmL.q);
m = size(pandaArm.ArmR.q);

% error = 0.1;
margin = 0.9;
for i = 1:n(1)
    if (pandaArm.jlmin(i) < 0)
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i)*margin, 0, 1, pandaArm.ArmL.q(i));
    else
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i)*margin, pandaArm.jlmin(i), 0, 1, pandaArm.ArmL.q(i));
    end
    if (pandaArm.jlmax(i) > 0)
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i)*margin, pandaArm.jlmax(i), 0, 1, pandaArm.ArmL.q(i));
    else
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i), pandaArm.jlmax(i)*margin, 0, 1, pandaArm.ArmL.q(i));
    end
    pandaArm.A.jl_left(i,i) = minBell + maxBell;
    
end

for i = 1:m(1)
    if (pandaArm.jlmin(i) < 0)
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i)*margin, 0, 1, pandaArm.ArmR.q(i));
    else
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i)*margin, pandaArm.jlmin(i), 0, 1, pandaArm.ArmR.q(i));
    end
    if (pandaArm.jlmax(i) > 0)
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i)*margin, pandaArm.jlmax(i), 0, 1, pandaArm.ArmR.q(i));
    else
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i), pandaArm.jlmax(i)*margin, 0, 1, pandaArm.ArmR.q(i));
    end
    pandaArm.A.jl_right(i,i) = minBell + maxBell;
    
end

pandaArm.A.jl = [pandaArm.A.jl_left zeros(7,7);
                 zeros(7,7)         pandaArm.A.jl_right] .* ActionTransition("JL", previous_action_list, current_action_list, mission.phase_time);

end