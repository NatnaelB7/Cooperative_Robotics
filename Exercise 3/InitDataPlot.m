function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);
    
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);
    plt.q2 = zeros(7, maxloops);
    plt.q_dot2 = zeros(7, maxloops);
    
    % Cartesian velocity
    plt.xdot1 = zeros(6, maxloops);
    plt.xdot2 = zeros(6, maxloops);

    plt.mission = zeros(1, maxloops);

    % Desired cartesian velocity
    plt.xdot1_tool = zeros(6, maxloops);
    plt.xdot2_tool = zeros(6, maxloops);
    
    % Desired cartesian velocity non-cooperative
    plt.xdot1_non_coop = zeros(6, maxloops);
    plt.xdot2_non_coop = zeros(6, maxloops);
    
    % Desired cartesian velocity cooperative
    plt.xdot1_coop = zeros(6, maxloops);
    plt.xdot2_coop = zeros(6, maxloops);

    % Joint limit activation functions
    plt.arm1.a.jl = zeros(7,maxloops);
    plt.arm2.a.jl = zeros(7,maxloops);

    % Minimum altitude activation functions 
    plt.arm1.a.ma = zeros(1,maxloops);
    plt.arm2.a.ma = zeros(1,maxloops);

    % Tool activation functions (move to)
    plt.arm1.a.tool = zeros(6, maxloops);
    plt.arm2.a.tool = zeros(6, maxloops);

    % Mutual motion activation function
    plt.arm1.a.coop = zeros(6, maxloops);
    plt.arm2.a.coop = zeros(6, maxloops);

    % Rigid constraint activation function
    plt.arm1.a.rc = zeros(6,maxloops);
    plt.arm2.a.rc = zeros(6,maxloops);

    % Cartesian positions
    plt.arm1.p = zeros(6, maxloops);
    plt.arm2.p = zeros(6, maxloops);

    % Cooperative weights
    plt.arm1.m = zeros(1,maxloops);
    plt.arm2.m = zeros(1,maxloops);

end
