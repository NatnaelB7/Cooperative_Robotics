function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own

% All Activation Functions 
figure(1), grid on;
title( "Activation Functions", "All The Activation Functions" );
hold on;
plot( plt.t, plt.A.zv, ':g' );
plot( plt.t, plt.A.ma, '-b' );
plot( plt.t, plt.A.ha, '-r' );
plot( plt.t, plt.A.al, '-m' );
plot( plt.t, plt.A.a, '-.b' );
plot( plt.t, plt.A.vp, '--b' );
plot( plt.t, plt.A.vo, '--r' );
plot( plt.t, plt.A.t, '-.k' );
hold off;
ylim( [0, 1.1] )          
legend( ...
    'zero velocity constraint', ...
    'minimum altitude', ...
    'horizontal attitude', ...
    'vehicle alignment to the target', ...
    'zero altitude', ...
    'vehicle position control -- x', ...
    '-- y', ...
    '-- z', ...
    'vehicle orientation control -- roll', ...
    '-- pitch', ...
    '-- yaw', ...
    'grasping task' )


% Minimum Altitude and Zero Altitude Control Task
figure(2), grid on;
title( "Activation Functions", "Minimum Altitude Vs Zero Altitude Control Task" );
hold on;
plot( plt.t, plt.A.ma, '-b' );
plot( plt.t, plt.A.a, '--b' );
hold off;
legend( ...
    'minimum altitude', ...
    'zero altitude' )

% Vehicle Position and Orientation 
figure(3);
subplot( 2, 1, 1 );
hold on, grid on;
plot( plt.t, plt.vpos(1, :), '-r' );
plot( plt.t, plt.vpos(2, :), '-b' );
plot( plt.t, plt.vpos(3, :), '-k' );
title( "Vehicle Position" )
legend( 'x', 'y', 'z' );
hold off;
subplot( 2, 1, 2 );
hold on, grid on;
plot( plt.t, plt.vorient(1, :), '-r' );
plot( plt.t, plt.vorient(2, :), '-b' );
plot( plt.t, plt.vorient(3, :), '-k' );
title( "Vehicle Orientation" )
legend( 'roll (x)', 'pitch (y)', 'yaw (z)' );
hold off;
sgtitle( "Vehicle Position and Orientation" );


% Desired Vehicle Position and Orientation 
figure(4);
subplot( 2, 1, 1 );
hold on, grid on;
plot( plt.t, plt.dot_vpos(1, :), '-r' );
plot( plt.t, plt.dot_vpos(2, :), '-b' );
plot( plt.t, plt.dot_vpos(3, :), '-k' );
title( "Desired Vehicle Position" )
legend( 'Xdot', 'Ydot', 'Zdot' );
hold off;
subplot( 2, 1, 2 );
hold on, grid on;
plot( plt.t, plt.dot_vorient(1, :), '-r' );
plot( plt.t, plt.dot_vorient(2, :), '-b' );
plot( plt.t, plt.dot_vorient(3, :), '-k' );
title( "Desired Vehicle Orientation" )
legend( 'Wx', 'Wy', 'Wx' );
hold off;
sgtitle( "Desired Vehicle Position and dotOrientation" );


% Vehicle Position Vs End-effector Position
figure(5);
subplot( 2, 1, 1 );
hold on, grid on;
plot( plt.t, plt.dot_vpos(1, :), ':b' );
plot( plt.t, plt.dot_vpos(2, :), '-b' );
plot( plt.t, plt.dot_vpos(3, :), '--b' );
% plot( plt.t, plt.vorient(1, :), ':r' );
% plot( plt.t, plt.vorient(2, :), '-r' );
% plot( plt.t, plt.vorient(3, :), '--r' );
title( "Desired position" )
legend( 'Xdot', 'Ydot', 'Zdot' );
hold off;
subplot( 2, 1, 2 );
hold on, grid on;
plot( plt.t, plt.q_dot );
title( "Configuration Rate" )
legend( 'qdot_1', 'qdot_2', 'qdot_3', 'qdot_4', 'qdot_5', 'qdot_6', 'qdot_7' );
hold off;
sgtitle( "Vehicle Velocity Vs. Manipulator Tool" );

end