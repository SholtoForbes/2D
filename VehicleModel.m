function [dfuel, Fueldt, a, q, M, Fd, Thrust, flapdeflection, Alpha, heating_rate, Q, rho] = VehicleModel(time, theta, V, v, mfuel, nodes,AoA_spline,flapdeflection_spline,Dragq_spline,ThrustF_spline,FuelF_spline, const)
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

StartingV = 0; % Calculate ablsolute height % THIS NEEDS TO BE CHANGED FOR VARIABLE HEIGHT

Vabs = V + StartingV; % Absolute vertical position

c = spline( Atmosphere(:,1),  Atmosphere(:,5), Vabs); % Calculate speed of sound using atmospheric data

rho = spline( Atmosphere(:,1),  Atmosphere(:,4), Vabs); % Calculate density using atmospheric data

q = 0.5 * rho .* (v_array .^2); % Calculating Dynamic Pressure



M = v_array./c; % Calculating Mach No (Descaled)



%-heating---------------------------
kappa = 1.7415e-4;
Rn = 1; %effective nose radius (m) (need to change this, find actual value)

heating_rate = kappa*sqrt(rho./Rn).*v_array.^3; %watts

Q = zeros(1,length(time));
Q(1) = 0;

for i = 1:length(dt_array)
    Q(i+1) = heating_rate(i)*dt_array(i) + Q(i);
end



% mstruct = 8755.1 - 994 + max(Q)/(10^5); %variable structural mass
% m = mfuel + mstruct;


[Fd, Alpha, flapdeflection] = OutForce(theta,M,q,m,AoA_spline,flapdeflection_spline,Dragq_spline);
% Fd = Fd*8/10;

% THRUST AND MOTION ==================================================================
% Thrust(1:nodes) =  50000;


if const == 1
Efficiency = zeros(1,length(q));
for i = 1:length(q)
    if q(i) < 50000
    Efficiency(i) = rho(i)/(50000*2/v_array(i)^2);

    else
    Efficiency(i) = 1;
    end
end

else
        
Efficiency = rho./(50000*2./v_array.^2); % linear rho efficiency, scaled to rho at 50000kpa
end

% Efficiency = q./50000; % linear q efficiency, this isnt really efficiency in fuel, scales fuel use as well.... more like thrust scaling
% Efficiency2 = q./50000.*(-(q-50000).^4.*6.25e-19 + 1);% test of quadratic dropoff (0.9 at 70kpa), used only for thrust (fuel will still use linear efficiency)
% Efficiency = atan(q/5000)/pi*2;
Efficiency4 = -((q-50000)./50000).^2 + 1; % this is an assumption of how the engine behaves

% Thrust(1:nodes) =  200000;
% Thrust = ThrustF_spline(M,Alpha).*Efficiency2;
% Thrust = ThrustF_spline(M,Alpha).*Efficiency;

% Acceleration ------------------------------------------------------------

% a = ((Thrust - (Fd + g*sin(theta))) ./ m ); % acceleration

%Fuel Cost ===========================================================================
% Efficiency

% Fueldt = FuelF(M,Alpha);
Fueldt = FuelF_spline(M,Alpha).*Efficiency;
% Fueldt = FuelF_spline(M,Alpha);

Isp = ThrustF_spline(M,Alpha)./FuelF_spline(M,Alpha); % this isnt quite Isp (doesnt have g) but doesnt matter

% Fueldt(1:nodes) = 4; % arbitrary

Thrust = Isp.*Fueldt;

% Thrust = Isp.*Fueldt.*Efficiency4;

fuelchange_array = -Fueldt(1:end-1).*dt_array ;


dfuel = sum(fuelchange_array); %total change in 'fuel' this is negative

a = ((Thrust - (Fd + g*sin(theta))) ./ m );


end








