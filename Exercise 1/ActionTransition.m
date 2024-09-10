function [a] = ActionTransition(task_name, previous_action, current_action, action_time)

if(ismember(task_name,previous_action) == 0 && ismember(task_name,current_action) && task_name == "ZV")
    a = 1;
elseif ((ismember(task_name, previous_action) && ismember(task_name, current_action) == 0) && task_name == "ZV")
    a = 0;

elseif (ismember(task_name,previous_action) && ismember(task_name,current_action))
    a = 1;
elseif (ismember(task_name,previous_action) == 0 && ismember(task_name,current_action))
    a = IncreasingBellShapedFunction(0, 1, 0, 1, action_time);
elseif (ismember(task_name,previous_action) && ismember(task_name,current_action) == 0)
    a = DecreasingBellShapedFunction(0, 1, 0, 1, action_time);    
else
    a = 0;
        
end

