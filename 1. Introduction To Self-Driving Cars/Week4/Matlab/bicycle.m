%% Init
clc
clear

%% Using ODE45
[t, states] = ode45(@bicycleModel, 0:0.01:10, [0; 0; 0; 0.0005]);

%% Results
x_vec = states(:, 1);
y_vec = states(:, 2);
plot(x_vec, y_vec);
axis equal