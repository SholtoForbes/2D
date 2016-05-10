function [dfuel, Fueldt, a, q, M, Fd, Thrust, flapdeflection, Alpha, heating_rate, Q, rho] = VehicleModel(time, theta, V, v, mfuel, nodes,AoA_spline,flapdeflection_spline,Drag_spline,ThrustF_spline,FuelF_spline, const,thetadot)
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

mstruct = 8755.1 - 994; % mass of everything but fuel from dawids work,added variable struct mass just under q calc
% mstruct = 8755.1 - 994-3000;

m = mfuel + mstruct;

%===================================================
%
% SECOND STAGE
%
%===================================================

%======================================================
% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments


Atmosphere = dlmread('atmosphere.txt'); % import data from atmosphere matrix

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


% Control

% determine aerodynamics necessary for trim
[Fd, Alpha, flapdeflection] = OutForce(theta,M,q,m,AoA_spline,flapdeflection_spline,Drag_spline,v,V);

% Fd = 1.1*Fd; % for L/D testing



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
%     Efficiency(i) = 1.2; 
    end
end

else
        
Efficiency = rho./(50000*2./v.^2); % linear rho efficiency, scaled to rho at 50000kpa
end


%Fuel Cost ===========================================================================
% Efficiency

% Fueldt = FuelF(M,Alpha);
Fueldt = FuelF_spline(M,Alpha).*Efficiency;
% Fueldt = FuelF_spline(M,Alpha);

Isp = ThrustF_spline(M,Alpha)./FuelF_spline(M,Alpha); % this isnt quite Isp (doesnt have g) but doesnt matter

% Fueldt(1:nodes) = 4; % arbitrary

Thrust = Isp.*Fueldt.*cos(theta); % Thrust in Direction of Motion (N)

% Thrust = Isp.*Fueldt.*Efficiency4;

fuelchange_array = -Fueldt(1:end-1).*dt_array ;


dfuel = sum(fuelchange_array); %total change in 'fuel' this is negative

v_H = v.*cos(Alpha);

a = ((Thrust - (Fd + (6.674e-11.*5.97e24./(V + 6371e3).^2 - v_H.^2./(V + 6371e3)).*sin(theta))) ./ m );


end








