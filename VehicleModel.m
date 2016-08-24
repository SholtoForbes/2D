function [dfuel, Fueldt, a, q, M, Fd, Thrust, flapdeflection, Alpha, rho,lift] = VehicleModel(time, theta, V, v, mfuel, nodes,scattered, grid, const,thetadot, Atmosphere)

% =======================================================
% Vehicle Model
% =======================================================

%Gravity
g = 9.81;

dt_array = time(2:end)-time(1:end-1); % Time change between each node pt

mstruct = 8755.1 - 994; % mass of everything but fuel from dawids work,added variable struct mass just under q calc

m = mfuel + mstruct;

%===================================================
%
% SECOND STAGE
%
%===================================================

%======================================================
% THIS IS VERY COMPUTATIONALLY EXPENSIVE TAKE OUT
% Atmosphere = dlmread('atmosphere.txt'); % import data from atmosphere matrix


c = spline( Atmosphere(:,1),  Atmosphere(:,5), V); % Calculate speed of sound using atmospheric data

rho = spline( Atmosphere(:,1),  Atmosphere(:,4), V); % Calculate density using atmospheric data

q = 0.5 * rho .* (v .^2); % Calculating Dynamic Pressure

M = v./c; % Calculating Mach No (Descaled)

%-heating---------------------------
kappa = 1.7415e-4;
Rn = 1; %effective nose radius (m) (need to change this, find actual value)

heating_rate = kappa*sqrt(rho./Rn).*v.^3; %watts

Q = zeros(1,length(time));
Q(1) = 0;

for i = 1:length(dt_array)
    Q(i+1) = heating_rate(i)*dt_array(i) + Q(i);
end

% Control =================================================================

% determine aerodynamics necessary for trim
[Fd, Alpha, flapdeflection,lift] = OutForce(theta,M,q,m,scattered,v,V,thetadot,time);

% Fd = 1.1*Fd; % for L/D testing COMMENT OUT IF NOT HIGH DRAG


% THRUST AND MOTION ==================================================================

if const == 1
    Efficiency = zeros(1,length(q));
    for i = 1:length(q)
        if q(i) < 50000
    %     if q(i) < 55000
    %     if q(i) < 45000
        Efficiency(i) = rho(i)/(50000*2/v(i)^2); % dont change this

        else
    %         Efficiency(i) = .9; % for 45kPa
        Efficiency(i) = 1; % for 50kPa
    %     Efficiency(i) = 1.1; % for 55kPa
    % %     Efficiency(i) = 1.2; 
        end
    end
else       
    Efficiency = rho./(50000*2./v.^2); % linear rho efficiency, scaled to rho at 50000kpa
end

%Fuel Cost ===========================================================================

Thrust = interp2(grid.Mgrid_eng2,grid.alpha_eng2,grid.T_eng,M,Alpha,'spline').*cos(deg2rad(Alpha)).*Efficiency;
Fueldt = interp2(grid.Mgrid_eng2,grid.alpha_eng2,grid.fuel_eng,M,Alpha,'spline').*Efficiency;

% Fueldt = scattered.fuel(M,Alpha).*Efficiency;
% 
% Thrust = scattered.T(M,Alpha).*cos(deg2rad(Alpha)).*Efficiency;% Thrust in Direction of Motion (N)

fuelchange_array = -Fueldt(1:end-1).*dt_array ;

dfuel = sum(fuelchange_array); %total change in 'fuel' this is negative

v_H = v.*cos(theta);

gravity = m.*(- 6.674e-11.*5.97e24./(V + 6371e3).^2 + v_H.^2./(V + 6371e3)); %Includes Gravity Variation and Centripetal Force 

a = ((Thrust - (Fd + gravity.*sin(theta))) ./ m );




% =========================================================================



end








