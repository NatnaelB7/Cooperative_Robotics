function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  
            [ang, lin] = CartError(uvms.wTv, uvms.wTgv);
            if (norm(ang) < 0.1 && norm(lin(1:2)) < 0.1)
                mission.phase = 2;
                mission.phase_time = 0;
                mission.previous_action = mission.current_action;
                mission.current_action = "landing";
                disp('Transitioning to Phase 2: Landing');
            end

        case 2
            [ang, lin] = CartError(uvms.vTg, uvms.vTt);
            if (norm(ang) < 0.1 && norm(lin(1:2)) < 0.1)
                mission.phase = 3;
                mission.phase_time = 0;
                mission.previous_action = mission.current_action;
                mission.current_action = "grasping";
                disp('Transitioning to Phase 3: Grasping');
            end
    end
end
