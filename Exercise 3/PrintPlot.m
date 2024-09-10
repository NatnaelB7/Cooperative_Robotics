function [ ] = PrintPlot(plt)

% some predefined plots
% you can add your own

%% Linear tool velocities

    fig = figure("Name","Linear tool velocities");
    subplot(2,1,1);
    hplot = plot(plt.t, (plt.xdot1(4:6,:)),plt.t, (plt.xdot2(4:6,:)));
    title("Linear Tool Velocities in <w>","FontSize",24);
    set(hplot, 'LineWidth', 2)
    lgd = legend("x_1","y_1","z_1", "x_2","y_2","z_2","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("[m/s]","FontSize",24);
    xlabel("Time [sec]","FontSize",24);
    fontsize(gca,22,"points");
    
    subplot(2,1,2);
    hplot = plot(plt.t, (plt.xdot1(4:6,:)) - (plt.xdot2(4:6,:)));
    title(["Differences Between Linear Tool Velocities","Arm1 - Arm2"],"FontSize",24);
    set(hplot, "LineWidth", 2);
    lgd = legend("x_1 - x_2", "y_1 - y_2", "z_1 - z_2","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("[m/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");


    %% Arm1 Desired Linear Velocities Vs Non-cooperative vs cooperative velocities 
    fig = figure("Name","Arm1 Desired Linear Velocities Vs Linear Non-cooperative vs cooperative velocities ");
    
    subplot(3,1,1);
    plot(plt.t(1:end), plt.xdot1_tool(4:6,:),"LineWidth", 2);
    title("Desired Linear Tool Velocities in <w> - Arm1", "FontSize", 24);
    lgd = legend("v_x","v_y","v_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("Velocities [m/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");
    

    subplot(3,1,2);
    hplot = plot(plt.t, (plt.xdot1_non_coop(4:6,:)));
    title("Linear Non-cooperative Tool Velocities in <w> - Arm1","FontSize",24);
    set(hplot, 'LineWidth', 2);
    lgd = legend("v_x","v_y","v_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("Velocities [m/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    subplot(3,1,3);
    hplot = plot(plt.t, (plt.xdot1_coop(4:6,:)));
    title("Linear Cooperative Tool Velocities in <w> - Arm1","FontSize",24);
    set(hplot, 'LineWidth', 2)
    lgd = legend("v_x","v_y","v_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("[m/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");


    %% Arm1 Desired Agular Velocities Vs Non-cooperative vs cooperative velocities 
    fig = figure("Name","Arm1 Desired Angular Vs Angular Non-cooperative vs cooperative velocities");

    subplot(3,1,1);
    plot(plt.t(1:end), plt.xdot1_tool(1:3,:),"LineWidth", 2);
    title("Desired Angular Tool Velocities in <w> - Arm1", "FontSize", 24);
    lgd = legend("\omega_x","\omega_y","\omega_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("Velocities [rad/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");
    

    subplot(3,1,2);
    hplot = plot(plt.t, (plt.xdot1_non_coop(1:3,:)));
    title("Non-cooperative Angular Tool Velocities in <w> - Arm1","FontSize",24);
    set(hplot, 'LineWidth', 2);
    lgd = legend("\omega_x","\omega_y","\omega_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("Velocities [rad/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");


    subplot(3,1,3);
    hplot = plot(plt.t, (plt.xdot1_coop(1:3,:)));
    title("Cooperative Angular Tool Velocities in <w> - Arm1","FontSize",24);
    set(hplot, 'LineWidth', 2)
    lgd = legend("\omega_x","\omega_y","\omega_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("Velocities [rad/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");


    %% Arm 2 Desired Linear Velocities vs Linear Non-cooperative Vs Cooperative Velocities 
    fig = figure("Name","Desired Linear vs Linear Non-cooperative Vs Cooperative Velocities");
    
    subplot(3,1,1);
    plot(plt.t(1:end), plt.xdot2_tool(4:6,:),"LineWidth", 2);
    title("Desired Linear Tool Velocities in <w> - Arm2", "FontSize", 24);
    lgd = legend("v_x","v_y","v_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("Velocities [m/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");
    

    subplot(3,1,2);
    hplot = plot(plt.t, (plt.xdot2_non_coop(4:6,:)));
    title("Non-cooperative Linear Tool Velocities in <w> - Arm2","FontSize",24);
    set(hplot, 'LineWidth', 2);
    lgd = legend("v_x","v_y","v_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("Velocities [m/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    subplot(3,1,3);
    hplot = plot(plt.t, (plt.xdot2_coop(4:6,:)));
    title("Cooperative Linar Tool Velocities in <w> - Arm2","FontSize",24);
    set(hplot, 'LineWidth', 2)
    lgd = legend("v_x","v_y","v_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("Velocities [m/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");


    %% Arm2 Desired Angular Velocities Vs Non-cooperative vs Cooperative Velocities
    fig = figure("Name","Desired vs Non-cooperative vs Cooperative Angular Velocities - Arm2");
    
    subplot(3,1,1);
    plot(plt.t(1:end), plt.xdot2_tool(1:3,:),"LineWidth", 2);
    title("Desired Tool Angular Velocities in <w> - Arm2", "FontSize", 24);
    lgd = legend("\omega_x","\omega_y","\omega_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("Velocities [rad/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    subplot(3,1,2);
    hplot = plot(plt.t, (plt.xdot2_non_coop(1:3,:)));
    title("Non-cooperative Tool Angular Velocities in <w> - Arm2","FontSize",24);
    set(hplot, 'LineWidth', 2);
    lgd = legend("\omega_x","\omega_y","\omega_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("Velocities [rad/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    subplot(3,1,3);
    hplot = plot(plt.t, (plt.xdot2_coop(1:3,:)));
    title("Cooperative Tool Angular Velocities in <w> - Arm2","FontSize",24);
    set(hplot, 'LineWidth', 2)
    lgd = legend("\omega_x","\omega_y","\omega_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    grid on;
    ylabel("Velocities [rad/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    %% Ioint Limit Activation Function
    fig = figure("Name", "Joint limit activation functions - Arm1");
    plot(plt.t, plt.arm1.a.jl, "LineWidth", 2);
    grid on;
    title("Joint limit activation functions - Arm1","FontSize",24);
    lgd = legend("Ajl1","Ajl2","Ajl3","Ajl4","Ajl5","Ajl6","Ajl7","Location","northeastoutside");
    fontsize(lgd,22,"points");
    ylabel("Activation","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    fig = figure("Name", "Joint limit activation functions - Arm2");
    plot(plt.t, plt.arm2.a.jl, "LineWidth", 2);
    grid on;
    title("Joint limit activation functions - Arm2","FontSize",24);
    lgd = legend("Ajl1","Ajl2","Ajl3","Ajl4","Ajl5","Ajl6","Ajl7","Location","northeastoutside");
    fontsize(lgd,22,"points");
    ylabel("Activation","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    %% Activation Functions
    fig = figure("Name", "Activation Functions - Arm1");
    plot(plt.t, plt.arm1.a.ma, ...
        plt.t, plt.arm1.a.tool(1,:), ...
        plt.t, plt.arm1.a.coop(1,:), ...% plt.t, plt.arm1.a.rc(1,:), ...
        "LineWidth", 2);
    grid on;
    title("Activation Functions - Arm1","FontSize",24);
    lgd = legend("MA", "T","MM", "Location","northeastoutside");
    fontsize(lgd,22,"points");
    ylabel("Activations","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    fig = figure("Name", "Activation Functions - Arm2");
    plot(plt.t, plt.arm2.a.ma, ...
        plt.t, plt.arm2.a.tool(1,:), ...
        plt.t, plt.arm2.a.coop(1,:), ...% plt.t, plt.arm2.a.rc(1,:), ...
        "LineWidth", 2);
    grid on;
    title("Activation Functions","FontSize",24);
    lgd = legend("MA", "T","MM", "Location","northeastoutside");
    fontsize(lgd,22,"points");
    ylabel("Activations","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    %% Cartesian Positions
    fig = figure("Name", "Cartesian Positions - Arm1");
    plot(plt.t, plt.arm1.p, "LineWidth", 2);
    grid on;
    title("Cartesian Positions in <w> - Arm1","FontSize",24);
    lgd = legend("x", "y", "z", "\theta_x", "\theta_y", "\theta_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    ylabel("Positions (linear [m/s]/angular [rad/s])","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    fig = figure("Name", "Cartesian Positions - Arm2");
    plot(plt.t, plt.arm2.p, "LineWidth", 2);
    grid on;
    title("Cartesian Positions in <w> - Arm2","FontSize",24);
    lgd = legend("x", "y", "z", "\theta_x", "\theta_y", "\theta_z","Location","northeastoutside");
    fontsize(lgd,22,"points");
    ylabel("Positions (linear [m/s]/angular [rad/s])","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    %% Cooperation Parameters
    fig = figure("Name", "Arm - mu");
    subplot(2,1,1);
    plot(plt.t, plt.arm1.m, plt.t, plt.arm2.m, "LineWidth", 2);
    grid on;
    title("\mu", "FontSize", 24);
    lgd = legend("\mu_{arm1}", "\mu_{arm2}", "Location","northeastoutside");
    fontsize(lgd,22,"points");
    ylabel("\mu","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    subplot(2,1,2);
    plot(plt.t, sqrt(sum(plt.xdot1_non_coop.^2,1)), plt.t, sqrt(sum(plt.xdot2_non_coop.^2,1)), "LineWidth", 2);
    grid on;
    title("Non-cooperative Velocities", "FontSize", 24);
    lgd = legend("arm1", "arm2", "Location","northeastoutside");
    fontsize(lgd,22,"points");
    ylabel("||Velocities||","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

    %% Cooperative Velocities
    % Check for change times
    change_times = find(diff(plt.mission));
    
    % Handle empty change_times
    if isempty(change_times)
        warning('No changes detected in plt.mission, skipping cooperative velocities plot.');
        return; % Skip the plot or handle appropriately
    end
    
    avg = sqrt(sum(((plt.xdot1_non_coop + plt.xdot2_non_coop)/2).^2,1));
    avg_w = sqrt(sum(plt.xdot1_coop.^2,1));
    avg_diff = avg - avg_w;
    
    fig = figure("Name", "Cooperative Velocities");
    subplot(2,1,1);
    plot(plt.t(change_times(1)+1:end), avg(change_times(1)+1:end), plt.t(change_times(1)+1:end), avg_w(change_times(1)+1:end), "LineWidth", 2);
    grid on;
    title("Norm Cooperative Velocities", "FontSize", 24);
    lgd = legend("average", "weighted average", "Location","northeastoutside");
    fontsize(lgd,22,"points");
    ylabel("Velocities [m/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");
    
    subplot(2,1,2);
    plot(plt.t(change_times(1)+1:end), avg_diff(change_times(1)+1:end), ...
        "LineWidth", 2);
    grid on;
    title(["Difference norm(average) - norm(weighted average)", "cooperative velocities"], ...
        "FontSize", 24);
    lgd = legend("difference", "Location","northeastoutside");
    fontsize(lgd,22,"points");
    ylabel("Difference [m/s]","FontSize", 24);
    xlabel("Time [sec]","FontSize", 24);
    fontsize(gca,22,"points");

end