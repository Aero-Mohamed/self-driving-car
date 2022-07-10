function [state] = bicycleModel(dt, currentState)
    
    Lr = 5; % distance from rear wheel to cg
    L  = 10; % distance from rear wheel to front wheel
    v  = 10; % Forward Speed 
    delta_dot = currentState(4); % rads/sec , rate of change of steering angle
    delta_rads = delta_dot * dt;
    % Slide Slip angle due to no slip condition on front and rear wheels
    beta = atan(Lr * tan(delta_rads)/L);
    state = [...
        v*cos(currentState(3) + beta); ...
        v*sin(currentState(3) + beta); ...
        v*cos(beta)*tan(delta_rads*dt)/L ; ...
        delta_dot ...
    ];

end