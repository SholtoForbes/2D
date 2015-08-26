function [dfuel, Fueldt, a, q, M, Fd, Thrust] = VehicleModel(time, theta, V, H, v, mfuel, nodes, communicator, communicator_trim)
% function [dfuel, v, m, q, M, v_array] = VehicleModel(time, theta, V, H, nodes)


enginedata = dlmread('engineoutput_matrix');

% =======================================================
% Vehicle Model
% =======================================================

%Gravity
g = 9.81;

dt_array = time(2:end)-time(1:end-1); % Time change between each node pt

dV_array = V(2:end)-V(1:end-1); % Vertical position change between each node pt

dH_array = H(2:end)-H(1:end-1); % horizontal position change between each node pt

v_array = v;

mstruct = 8755.1 - 994; % mass of everything but fuel from dawids work

m = mfuel + mstruct;

%===================================================
%
% SECOND STAGE
%
%===================================================

%neglecting AoA, fixed reference frame

% mdot = -1.;
% mdot = 0.; 
% m = zeros(1,nodes-1);
% m(1) = 5000; 
% for i = 2:nodes
%     m(i) = m(i-1) + mdot*dt_array(i-1);
% end 

%======================================================
% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments

Iy = 1.; %%%% CHANGE THIS

% Out_force = dlmread('out_force.txt'); % import data from force matrix

Atmosphere = dlmread('atmosphere.txt'); % import data from atmosphere matrix

StartingV = 0; % Calculate ablsolute height % THIS NEEDS TO BE CHANGED FOR VARIABLE HEIGHT

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

S = 60;  % Planform area - this needs to be updated
[Fd, Alpha] = OutForce(theta,M,q,m,S, communicator, communicator_trim);


% THRUST AND MOTION ==================================================================

% Thrust 
% Thrust(1:nodes) =  50000;

Efficiency = rho./(50000*2./v_array.^2); % linear q efficiency, this is not exactly right
% Efficiency = 1;

% Thrust = griddata(enginedata(:,1), enginedata(:,2), enginedata(:,3), M, Alpha).*Efficiency; % thrust from engine data 
% Thrust = griddata(enginedata(:,1), enginedata(:,2), enginedata(:,3), M, Alpha);
% Thrust(1:nodes) =  200000;

ThrustF= scatteredInterpolant(enginedata(:,1),enginedata(:,2),enginedata(:,3)); %test extrapolator for engine data
% Thrust = ThrustF(M,Alpha).*Efficiency;
Thrust = ThrustF(M,Alpha).*Efficiency;

% Acceleration ------------------------------------------------------------

a = ((Thrust - (Fd + g*sin(theta))) ./ m ); % acceleration

%Fuel Cost ===========================================================================
% Efficiency

% Efficiency = gaussmf(q,[10000 50000]); this efficiency is other way round
% than above


%Fuel rate of change
% Fueldt = Thrust ./ Efficiency; % Temporary fuel rate of change solution, directly equated to thrust (should give correct efficiency result, but cannot analyse total fuel change accurately)

FuelF= scatteredInterpolant(enginedata(:,1),enginedata(:,2),enginedata(:,4)); %test extrapolator for engine data
% Fueldt = FuelF(M,Alpha);
Fueldt = FuelF(M,Alpha);

% Fueldt = griddata(enginedata(:,1), enginedata(:,2), enginedata(:,4), M, Alpha); % mass flow rate from engine data
% Fueldt = griddata(enginedata(:,1), enginedata(:,2), enginedata(:,4), M, Alpha)./ Efficiency;

fuelchange_array = -Fueldt(1:end-1).*dt_array ;

dfuel = sum(fuelchange_array); %total change in 'fuel' this is negative

end









