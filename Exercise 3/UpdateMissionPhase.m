function [pandaArm1, pandaArm2, mission] = UpdateMissionPhase(pandaArm1, pandaArm2, mission)    
        switch mission.phase
            case 1  % Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                % max error: 1/10 cm and 1deg
                [ang1, lin1] = CartError(pandaArm1.wTg, pandaArm1.wTt);
                [ang2, lin2] = CartError(pandaArm2.wTg, pandaArm2.wTt);

                if(norm(ang1) <= deg2rad(1) && norm(lin1) <= 0.001 && norm(ang2) <= deg2rad(1) && norm(lin2) <= 0.001)
                     mission.phase = 2;
                     mission.phase_time = 0;                     
                     mission.prev_action = mission.current_action;
                     mission.current_action = "coop_manip";                 
                end
                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task
                % max error: 1 cm and 3deg
                [ang1, lin1] = CartError(pandaArm1.wTog, pandaArm1.wTo);                
                [ang2, lin2] = CartError(pandaArm2.wTog, pandaArm2.wTo);

                 if(norm(ang1) <= deg2rad(3) && norm(lin1) <= 0.01 && norm(ang2) <= deg2rad(3) && norm(lin2) <= 0.01)
                     mission.phase = 3;                     
                     mission.phase_time = 0;
                     mission.prev_action = mission.current_action;                     
                     mission.current_action = "end_motion";
                end


            case 3 % Finish motion
                disp("Simulation Stopped");
        end 
end


