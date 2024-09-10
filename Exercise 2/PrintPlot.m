function [ ] = PrintPlot( plt, pandaArms )

% some predefined plots
% you can add your own

%% Left Arm
fig = figure("Name","Left Arm Joint Limits Activation");
plot(plt.t(1:end), plt.armL.a_jl,"LineWidth", 2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Join Limits Activation", "FontSize", 24);
fontsize(gca,22,"points");
title(["Left Arm Activation Function", "Joint Limits"], "FontSize",24);
lgd = legend("Aj1","Aj2","Aj3","Aj4","Aj5","Aj6","Aj7","Location","northeastoutside");
fontsize(lgd,22,"points");


fig = figure("Name","Left Arm Activations");
plot(plt.t(1:end), plt.armL.a_ma,...
    plt.t(1:end), plt.armL.a_tool(1,:), ...
    plt.t(1:end), plt.a_rc(1,:), ...
    "LineWidth",2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Activations", "FontSize", 24);
fontsize(gca,22,"points");
title("Left Arm Activation Function", "FontSize",24);
lgd = legend("MA","T","RC","Location","northeastoutside");
fontsize(lgd,22,"points");
ylim([-0.1 1.1]);


fig = figure("Name", "Left Arm Desired cartesian velocities");
subplot(2,1,1);
plot(plt.t(1:end), plt.armL.xdot_tool,"LineWidth", 2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Velocities [m/s] [rad/s]", "FontSize", 24);
fontsize(gca,22,"points");
title("Left Arm Desired Cartesian Velocities in <w>", "FontSize",24);
lgd = legend("v_x","v_y","v_z","omega_x","omega_y","omega_z","Location","northeastoutside");
fontsize(lgd,22,"points");


fig = figure("Name", "Left Arm Cartesian velocities");
subplot(2,1,1);
plot(plt.t(1:end), plt.armL.xdot,"LineWidth", 2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Velocities [m/s] [rad/s]", "FontSize", 24);
fontsize(gca,22,"points");
title("Left Arm Cartesian Velocities in <w>", "FontSize",24);
lgd = legend("v_x","v_y","v_z","omega_x","omega_y","omega_z","Location","northeastoutside");
fontsize(lgd,22,"points");


%% Right Arm
fig = figure("Name","Right Arm Joint Limits Activation");
plot(plt.t(1:end), plt.armR.a_jl,"LineWidth", 2);
grid on;
xlabel("Time [sec]");
ylabel("Join Limits Activation");
fontsize(gca,22,"points");
title(["Right Arm Activation Function", "Joint Limits"], "FontSize",24);
lgd = legend("Aj1","Aj2","Aj3","Aj4","Aj5","Aj6","Aj7","Location","northeastoutside");
fontsize(lgd,22,"points");


fig = figure("Name","Right Arm Activations");
plot(plt.t(1:end), plt.armR.a_ma,...
    plt.t(1:end), plt.armR.a_tool(1,:), ...
    plt.t(1:end), plt.a_rc(1,:), ...              % plt.a_rc(1,:)
    "LineWidth",2);
grid on;
xlabel("Time [s]", "FontSize",24);
ylabel("Activations", "FontSize", 24);
fontsize(gca,22,"points");
title("Right Arm Activation Function", "FontSize",24);
lgd = legend("MA","T","RC","Location","northeastoutside");
fontsize(lgd,22,"points");
ylim([-0.1 1.1]);


fig = figure("Name", "Right Arm Desired cartesian velocities");
subplot(2,1,1);
plot(plt.t(1:end), plt.armL.xdot_tool,"LineWidth", 2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Velocities [m/s] [rad/s]", "FontSize", 24);
fontsize(gca,22,"points");
title("Right Arm Desired Cartesian Velocities in <w>", "FontSize",24);
lgd = legend("v_x","v_y","v_z","omega_x","omega_y","omega_z","Location","northeastoutside");
fontsize(lgd,22,"points");


fig = figure("Name", "Right Arm Cartesian velocities");
subplot(2,1,2);
plot(plt.t(1:end), plt.armR.xdot,"LineWidth", 2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Velocities [m/s] [rad/s]", "FontSize", 24);
fontsize(gca,22,"points");
title("Right Arm Cartesian Velocities in <w>", "FontSize",24);
lgd = legend("v_x","v_y","v_z","omega_x","omega_y","omega_z","Location","northeastoutside");
fontsize(lgd,22,"points");


end