function [dfuel, Fueldt, a, q, M, Fd, Thrust, flapdeflection, Alpha, rho,lift, Penalty] = VehicleModel(time, theta, V, v, mfuel, nodes,scattered, grid, const,thetadot, Atmosphere)
% t1 = cputime;
% =======================================================
% Vehicle Model
% =======================================================

eta = .0*ones(1,length(time));

% eta = 0.3 - 0.0001*time;

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

%Rotational Coordinates =================================================
%=================================================

global xi
global phi
global zeta
xi = zeros(1,length(time));
phi = zeros(1,length(time));
zeta = zeros(1,length(time));

zeta(1) = deg2rad(64);

r = V + 6371000;
i= 1;
[xidot(i),phidot(i),zetadot(i), lift_search(i)] = RotCoords(r(i),xi(i),phi(i),theta(i),v(i),zeta(i),m(i),eta(i), thetadot(i),const);

for i = 2:length(time)
xi(i) = xi(i-1) + xidot(i-1)*(time(i) - time(i-1));
phi(i) = phi(i-1) + phidot(i-1)*(time(i) - time(i-1));
zeta(i) = zeta(i-1) + zetadot(i-1)*(time(i) - time(i-1));

[xidot(i),phidot(i),zetadot(i), lift_search(i)] = RotCoords(r(i),xi(i),phi(i),theta(i),v(i),zeta(i),m(i),eta(i), thetadot(i),const);
end


% Aero Data =============================================================
c = spline( Atmosphere(:,1),  Atmosphere(:,5), V); % Calculate speed of sound using atmospheric data

rho = spline( Atmosphere(:,1),  Atmosphere(:,4), V); % Calculate density using atmospheric data

q = 0.5 * rho .* (v .^2); % Calculating Dynamic Pressure

M = v./c; % Calculating Mach No (Descaled)

%-heating---------------------------
% kappa = 1.7415e-4;
% Rn = 1; %effective nose radius (m) (need to change this, find actual value)
% 
% heating_rate = kappa*sqrt(rho./Rn).*v.^3; %watts
% 
% Q = zeros(1,length(time));
% Q(1) = 0;
% 
% for i = 1:length(dt_array)
%     Q(i+1) = heating_rate(i)*dt_array(i) + Q(i);
% end

% Control =================================================================

% % determine aerodynamics necessary for trim
[Fd, Alpha, flapdeflection,lift] = OutForce(theta,M,q,m,scattered,v,V,thetadot,time, lift_search);
% Alpha
if const == 14
    Fd = 1.1*Fd; % for L/D testing 
end

% THRUST AND MOTION ==================================================================
if const == 1 || const == 14
    Efficiency = zeros(1,length(time));
    Penalty = zeros(1,length(time));
    for i = 1:length(time)
        if q(i) < 50000
            Efficiency(i) = rho(i)/(50000*2/v(i)^2); % dont change this
        else
            Efficiency(i) = 1; % for 50kPa
            Penalty(i) = q(i)/50000-1; 
        end
    end
elseif const == 12
    Efficiency = zeros(1,length(time));
    Penalty = zeros(1,length(time));
    for i = 1:length(time)
        if q(i) < 55000
            Efficiency(i) = rho(i)/(50000*2/v(i)^2); % dont change this
        else
            Efficiency(i) = 1.1; % for 55kPa
            Penalty(i) = q(i)/55000-1; 
        end
    end
elseif const == 13
    Efficiency = zeros(1,length(time));
    Penalty = zeros(1,length(time));
    for i = 1:length(time)
        if q(i) < 45000
            Efficiency(i) = rho(i)/(50000*2/v(i)^2); % dont change this
        else
            Efficiency(i) = .9; % for 45kPa
            Penalty(i) = q(i)/45000-1; 
        end
    end
elseif const == 3 || const == 31
    Efficiency = rho./(50000*2./v.^2); % linear rho efficiency, scaled to rho at 50000kpa
    Penalty = 0;
end
    
%Fuel Cost ===========================================================================

for i = 1:length(time)
    if Alpha(i) > 0 && Alpha(i) < 6
        Thrust(i) = interp2(grid.Mgrid_eng2,grid.alpha_eng2,grid.T_eng,M(i),Alpha(i),'spline').*cos(deg2rad(Alpha(i))).*Efficiency(i);
        Fueldt(i) = interp2(grid.Mgrid_eng2,grid.alpha_eng2,grid.fuel_eng,M(i),Alpha(i),'spline').*Efficiency(i);
    else
        Thrust(i) =  scattered.T(M(i),Alpha(i)).*cos(deg2rad(Alpha(i))).*Efficiency(i);
        Fueldt(i) =  scattered.fuel(M(i),Alpha(i)).*Efficiency(i);
    end
end

fuelchange_array = -Fueldt(1:end-1).*dt_array ;

dfuel = sum(fuelchange_array); %total change in 'fuel' this is negative

v_H = v.*cos(theta);

gravity = m.*(- 6.674e-11.*5.97e24./(V + 6371e3).^2 + v_H.^2./(V + 6371e3)); %Includes Gravity Variation and Centripetal Force 

a = ((Thrust - (Fd + gravity.*sin(theta))) ./ m );

% =========================================================================

% time = cputime - t1 
end








