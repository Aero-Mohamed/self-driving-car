function [vec_earth] = body2Earth2D(vec_body, theta_rads, bodyAxisOrigin)
    % Input Angle in rads
    % Earth Frame is Inertial Frame
    Trans_B2E = [ ...
        cos(theta_rads) sin(theta_rads); ...
        -sin(theta_rads) cos(theta_rads)];
    vec_earth = Trans_B2E*vec_body + bodyAxisOrigin;
end