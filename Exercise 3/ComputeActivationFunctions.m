function [pandaArm] = ComputeActivationFunctions(pandaArm,mission)

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
         pandaArm.A.tool = eye(6) .* ActionTransition("T", previous_action_list, current_action_list, mission.phase_time);
    case 2 % Move the object holding it firmly
        % Mutual Motion 
        pandaArm.A.coop = eye(6) .* ActionTransition("MM", previous_action_list, current_action_list, mission.phase_time);
        % Rigid Grasp Constraint
        pandaArm.A.rc = eye(6) .* ActionTransition("RC", previous_action_list, current_action_list, mission.phase_time);
        % Move-To
        pandaArm.A.tool = eye(6) .* ActionTransition("T", previous_action_list, current_action_list, mission.phase_time);
    case 3 % STOP any motion 
        pandaArm.A.coop = eye(6) .* ActionTransition("MM", previous_action_list, current_action_list, mission.phase_time);
        pandaArm.A.rc = eye(6) .* ActionTransition("RC", previous_action_list, current_action_list, mission.phase_time);
        pandaArm.A.tool = eye(6) .* ActionTransition("T", previous_action_list, current_action_list, mission.phase_time);
end

% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
pandaArm.A.ma = DecreasingBellShapedFunction(0.15, 0.20, 0, 1, pandaArm.wTt(3,4)) .* ActionTransition("MA", previous_action_list, current_action_list, mission.phase_time);

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error

n = size(pandaArm.q);
% error = 0.1;
margin = 0.9;
for i = 1:n(1)
    if (pandaArm.jlmin(i) < 0)
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i)*margin, 0, 1, pandaArm.q(i));
    else
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i)*margin, pandaArm.jlmin(i), 0, 1, pandaArm.q(i));
    end
    if (pandaArm.jlmax(i) > 0)
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i)*margin, pandaArm.jlmax(i), 0, 1, pandaArm.q(i));
    else
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i), pandaArm.jlmax(i)*margin, 0, 1, pandaArm.q(i));
    end
    pandaArm.A.jl(i,i) = minBell + maxBell;
    
end

pandaArm.A.jl = pandaArm.A.jl .* ActionTransition("JL", previous_action_list, current_action_list, mission.phase_time);
    
end