function Alpha = OutForce(theta,M,q,m)

% Out_force interpolator

% theta = 0.78
% 
% M = 5.3
% 
% q = 53000
% 
% Thrust = 50000
% 
% m = 5000
gravity = m*9.81;

Out_force = dlmread('out_force.txt'); % Imports from force matrix


q_array = Out_force(:,2);
M_array = Out_force(:,3);
F_sum =  Out_force(:,8)  - gravity*cos(theta) ;% Sum of v

Alpha_array = Out_force(:,5);


Alpha = griddata(q_array,M_array,F_sum, Alpha_array, q, M, 0);