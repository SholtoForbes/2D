function [dfuel, Fueldt, a, q, M, Fd, Thrust, flapdeflection, Alpha] = VehicleModel(time, theta, V, v, mfuel, nodes,AoA_spline,flapdeflection_spline,Dragq_spline,ThrustF_spline,FuelF_spline)
% function [dfuel, v, m, q, M, v_array] = VehicleModel(time, theta, V, H, nodes)




% =======================================================
% Vehicle Model
% =======================================================

%Gravity
g = 9.81;

dt_array = time(2:end)-time(1:end-1); % Time change between each node pt

% dV_array = V(2:end)-V(1:end-1); % Vertical position change between each node pt
% 
% dH_array = H(2:end)-H(1:end-1); % horizontal position change between each node pt

v_array = v;

mstruct = 8755.1 - mfuel(1); % mass of everything but fuel from dawids work

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

q = 0.5 * rho .* (v_array .^2); % Calculating Dynamic Pressure

M = v_array./c; % Calculating Mach No (Descaled)

[Fd, Alpha, flapdeflection] = OutForce(theta,M,q,m,AoA_spline,flapdeflection_spline,Dragq_spline);


% THRUST AND MOTION ==================================================================

% Thrust(1:nodes) =  50000;

% Efficiency = rho./(50000*2./v_array.^2); % linear rho efficiency, scaled to rho at 50000kpa, this is not exactly right
Efficiency = q./50000; % linear q efficiency
% Efficiency = 1;

% Thrust(1:nodes) =  200000;
Thrust = ThrustF_spline(M,Alpha).*Efficiency;

% Acceleration ------------------------------------------------------------

a = ((Thrust - (Fd + g*sin(theta))) ./ m ); % acceleration

%Fuel Cost ===========================================================================
% Efficiency

% Efficiency = gaussmf(q,[10000 50000]); this efficiency is other way round
% than above


%Fuel rate of change
% Fueldt = Thrust ./ Efficiency; % Temporary fuel rate of change solution, directly equated to thrust (should give correct efficiency result, but cannot analyse total fuel change accurately)


% Fueldt = FuelF(M,Alpha);
Fueldt = FuelF_spline(M,Alpha).*Efficiency;

% Fueldt = griddata(enginedata(:,1), enginedata(:,2), enginedata(:,4), M, Alpha); % mass flow rate from engine data
% Fueldt = griddata(enginedata(:,1), enginedata(:,2), enginedata(:,4), M, Alpha)./ Efficiency;

fuelchange_array = -Fueldt(1:end-1).*dt_array ;

dfuel = sum(fuelchange_array); %total change in 'fuel' this is negative

end









