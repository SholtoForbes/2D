function [dfuel, Fueldt, a, m, q, M, Fd] = VehicleModel(time, theta, V, H, v, nodes, communicator, communicator_trim)
% function [dfuel, v, m, q, M, v_array] = VehicleModel(time, theta, V, H, nodes)


% =======================================================
% Vehicle Model
% =======================================================

%Gravity
g = 9.81;

dt_array = time(2:end)-time(1:end-1); % Time change between each node pt

dV_array = V(2:end)-V(1:end-1); % Vertical position change between each node pt

dH_array = H(2:end)-H(1:end-1); % horizontal position change between each node pt

v_array = v;

%===================================================
%
% SECOND STAGE
%
%===================================================

%neglecting AoA, fixed reference frame

% mdot = -1.;
mdot = 0.; 
m = zeros(1,nodes-1);
m(1) = 5000; 
for i = 2:nodes
    m(i) = m(i-1) + mdot*dt_array(i-1);
end 

%======================================================
% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments

Iy = 1.; %%%% CHANGE THIS

Out_force = dlmread('out_force.txt'); % import data from force matrix

Atmosphere = dlmread('atmosphere.txt'); % import data from atmosphere matrix

StartingV = 27000; % Calculate ablsolute height % THIS NEEDS TO BE CHANGED FOR VARIABLE HEIGHT

Vabs = V + StartingV; % Absolute vertical position

c = spline( Atmosphere(:,1),  Atmosphere(:,5), Vabs); % Calculate speed of sound using atmospheric data

rho = spline( Atmosphere(:,1),  Atmosphere(:,4), Vabs); % Calculate density using atmospheric data

%an initial interpolator for the force values at a fixed Arot, alpha and
%dynamic pressure (0,  -0.0174532925199 (negative up) , 45000.0)
M_array = [4.5 , 5. , 5.5]; 

% Placeholder vehicle model values
% Fd_array = [-36427.6593981 , -42995.3909773 , -50209.1507264];
% Flift_array = [ 26851.676829 , 25865.7310572 , 24420.6025981 ];
% My_array = [305002.235162 , 256125.242712 , 196654.950117 ];

q = 0.5 * rho .* (v_array .^2); % Calculating Dynamic Pressure

M = v_array./c; % Calculating Mach No (Descaled)

% For each alpha, spline force results for current dynamic pressure and
% Mach no

% Fd = zeros(1,nodes);
% Alpha = zeros(1,nodes);
% for i = 1:nodes
%     S = 60;  % Planform area - this needs to be updated
%     theta_temp = theta(i);
%     M_temp = M(i);
%     q_temp = q(i);
%     m_temp = m(i);
%     
%     [Alpha(i), Fd(i) ,pitchingmoment] = OutForce(theta_temp,M_temp,q_temp,m_temp,S, communicator, communicator_trim);
% end

% Alpha
% theta
% M
% Fd

Fd = 25000;


% THRUST AND MOTION ==================================================================

% Fdtemp = spline(M_array, Fd_array, M);
% Fd = Fdtemp./(50000./q); % this is an attempt to implement change in drag with q
% Flift = spline(M_array, Flift_array, M)  ;
% My = spline(M_array, My_array, M)  ;

% Thrust 
Thrust(1:nodes) =  90000;

% Acceleration ------------------------------------------------------------

a = ((Thrust - (Fd + g*sin(theta))) ./ m ); % acceleration

%Fuel Cost ===========================================================================
% Efficiency
% NOTE DYNAMIC PRESSURE DOES NOT CHANGE DRAG IN FORCE FILE INGO GAVE ME, so
% making the efficiency only reliant on a specific dynamic pressure is
% nonsensical

Efficiency = gaussmf(q,[5000 50000]);

%Fuel rate of change
Fueldt = Thrust ./ Efficiency; % Temporary fuel rate of change solution, directly equated to thrust (should give correct efficiency result, but cannot analyse total fuel change accurately)

fuelchange_array = -Fueldt(1:end-1).*dt_array ;

dfuel = sum(fuelchange_array); %total change in 'fuel' this is negative

end









