function [state] = twoWheeledRobotKinematicModel(~, currentState)
    % w1: Wheel 1 rotation Velocity
    % r : Wheels Raduis
    % L : half distance between two wheels
    % psi_dot: heading rate of change 
    % currentState: [x, y, psi]
    r = 1;
    L = 5;
    w1 = 10;
    w2 = 12;
    v1 = r*w1;
    v2 = r*w2;
    v = (v1 + v2)/2;
    psi_dot = (v1 - v2)/2/L;
    
    
    state = [...
        v*cos(currentState(3)); ...
        v*sin(currentState(3)); ...
        psi_dot ...
    ];
end

