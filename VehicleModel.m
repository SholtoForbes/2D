function [dfuel, Fueldt, a, m, q, M, Fd] = VehicleModel(time, theta, V, H, v, nodes)
% function [dfuel, v, m, q, M, v_array] = VehicleModel(time, theta, V, H, nodes)


% =======================================================
% Vehicle Model
% =======================================================
MultiStage = 0;


if MultiStage == 1    
    nodes_transition = nodes(1);
    nodes = sum(nodes);
end


%Gravity
g = 9.81;


dt_array = time(2:end)-time(1:end-1); % Time change between each node pt

dV_array = V(2:end)-V(1:end-1); % Vertical position change between each node pt

dH_array = H(2:end)-H(1:end-1); % horizontal position change between each node pt

%velocity primal
v_array = v;
% v_array = sqrt(dV_array.^2 + dH_array.^2) ./ dt_array; % calculate velocity array using 'previous iteration'


%neglecting AoA, fixed reference frame

% VARIABLE MASS TEST
if MultiStage == 1
m = zeros(1,nodes-1);
% define mass flow rates
mdot1 = 0.;
mdot2 = 0.;

% Initial stage masses
m(1) = 5000;
m(nodes_transition+1) = 5000;

% NEED TO CHANGE THIS TO SEPARATE AT A DISTINCT CONDITION
for i = 2:nodes_transition
    m(i) = m(i-1) + mdot1*dt_array(i-1);
end

for i = nodes_transition+2:nodes
    m(i) = m(i-1) + mdot2*dt_array(i-1);
end

else
% mdot = -1.;
mdot = 0.; 
m = zeros(1,nodes-1);
m(1) = 5000; 
for i = 2:nodes
    m(i) = m(i-1) + mdot*dt_array(i-1);
end 
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
Fd_array = [-36427.6593981 , -42995.3909773 , -50209.1507264];
Flift_array = [ 26851.676829 , 25865.7310572 , 24420.6025981 ];
My_array = [305002.235162 , 256125.242712 , 196654.950117 ];

q = 0.5 * rho .* (v_array .^2); % Calculating Dynamic Pressure

M = v_array./c; % Calculating Mach No (Descaled)



% For each alpha, spline force results for current dynamic pressure and
% Mach no

% This needs to be implemented with a more robust vehicle model, WILL ALL
% BE NaN for a vehicle model with not enough data
% for i=1:nodes-1
% theta_temp = theta(i); 
% M_temp = M(i);
% q_temp = q(i);
% m_temp = m(i);
%     
% AoA(i) = OutForce(theta_temp,M_temp,q_temp,m_temp);
% 
% end
% rho
% q
% M
% AoA

% THRUST AND MOTION ==================================================================
if MultiStage == 1
    Fdtemp(1:nodes_transition) = spline(M_array, Fd_array, M(1:nodes_transition));
    Fd = Fdtemp./(50000./q(1:nodes_transition)); % Modified drag force to include variation with q
    Fd(nodes_transition+1:nodes) = spline(M_array, Fd_array, M(nodes_transition+1:nodes)); % drag after transition
else
    Fdtemp = spline(M_array, Fd_array, M);
    Fd = Fdtemp./(50000./q); % this is an attempt to implement change in drag with q
    Flift = spline(M_array, Flift_array, M)  ;
    My = spline(M_array, My_array, M)  ;
end


% Thrust 
if MultiStage == 1
    Thrust(1:nodes_transition) =  180000;
    Thrust(nodes_transition+1:nodes) =  180000; 
else
    Thrust(1:nodes) =  180000;
end

% Acceleration ------------------------------------------------------------

a = ((Thrust - (- Fd + g*sin(theta))) ./ m ); % acceleration
% a = Thrust./Thrust * 4.5; %test constant acceleration

%Fuel Cost ===========================================================================
% Efficiency
% NOTE DYNAMIC PRESSURE DOES NOT CHANGE DRAG IN FORCE FILE INGO GAVE ME, so
% making the efficiency only reliant on a specific dynamic pressure is
% nonsensical

if MultiStage == 1
    Efficiency2 = gaussmf(q(1:nodes_transition),[5000 50000]);
    Efficiency3 = gaussmf(q(1:nodes_transition),[5000 50000]);
else
    % Efficiency = 1;
    Efficiency = gaussmf(q,[5000 50000]);
end

%Fuel rate of change
if MultiStage == 1
    Fueldt(1:nodes_transition) = Thrust(1:nodes_transition) ./ Efficiency2;
    Fueldt(nodes_transition+1:nodes) = Thrust(nodes_transition+1:nodes) ./ Efficiency3;
else
    Fueldt = Thrust ./ Efficiency; % Temporary fuel rate of change solution, directly equated to thrust (should give correct efficiency result, but cannot analyse total fuel change accurately)
end

fuelchange_array = -Fueldt(1:end-1).*dt_array ;

dfuel = sum(fuelchange_array); %total change in 'fuel' this is negative












