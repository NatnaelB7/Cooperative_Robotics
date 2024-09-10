function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);

    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);

    plt.p = zeros(6, maxloops);
    plt.p_dot = zeros(6, maxloops);

    
    % ======ACTIVATION FUNCTIONS======
    % minimum altitude task
    plt.A.ma = zeros(1, maxloops);

    % horizontal attitude task
    plt.A.ha = zeros(1, maxloops);

    % zero altitude control task
    plt.A.a = zeros(1, maxloops);

    % position control task
    plt.A.vp = zeros(3, maxloops);

    % orientation control task
    plt.A.vo = zeros(3, maxloops);
    
    % manipulation tool task 
    plt.A.t = zeros(6, maxloops);

    % zero velocity constraint task
    plt.A.zv = zeros(1, maxloops);

    % vehicle alignment to a target task
    plt.A.al = zeros(1, maxloops);

    % grasping task
    plt.A.g = zeros(6, maxloops);
    
    % =====VELOCITIES======
    % Desired cartesian velocity
    plt.xdot_t = zeros(6, maxloops);
    % Cartesian velocity
    plt.xdot = zeros(6, maxloops);


    % plt.xdot_jl = zeros(7, maxloops);
    % plt.xdot_mu = zeros(1, maxloops);
    plt.a = zeros(11, maxloops);

     % Vehicle Position and Orientation
    plt.vpos = zeros(3, maxloops);
    plt.vorient = zeros(3, maxloops);
    
    % Desired Vehicle Position and Orientation
    plt.dot_vpos = zeros(3, maxloops);
    plt.dot_vorient = zeros(3, maxloops);

end

