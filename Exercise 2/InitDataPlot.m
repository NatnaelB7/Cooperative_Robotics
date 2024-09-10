function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);
    
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);
    plt.q2 = zeros(7, maxloops);
    plt.q_dot2 = zeros(7, maxloops);
    
    % LEFT ARM

    % activation joint limits
    plt.armL.a_jl = zeros(7, maxloops);
    % activation minimum altitude 
    plt.armL.a_ma = zeros(1, maxloops);
    % activation tool 
    plt.armL.a_tool = zeros(6, maxloops);
    % desired cartesian velocity
    plt.armL.xdot_tool = zeros(6, maxloops);
    % cartesian velocity
    plt.armL.xdot = zeros(6, maxloops);

    % RIGHT ARM

    % activation joint limits 
    plt.armR.a_jl = zeros(7, maxloops);
    % activation minimum altitude 
    plt.armR.a_ma = zeros(1, maxloops);
    % activation tool 
    plt.armR.a_tool = zeros(6, maxloops);
    % desired cartesian velocity
    plt.armR.xdot_tool = zeros(6, maxloops);
    % cartesian velocity
    plt.armR.xdot = zeros(6, maxloops);

    % activation rigid constraint
    plt.a_rc = zeros(6, maxloops);

    % mission phase
    plt.phase = zeros(1, maxloops);

end

