function XDOT = TwoStage2d(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics

% This uses velocity calculated in the Cost file

% This file is calculated after the Cost file in the iterative process 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% global v
global a
global Fueldt
global heating_rate
global const
global drho
global rho

%changed notation to horizontal and vertical, x and y in plane of vehicle
V = primal.states(1,:) ; 

v = primal.states(2,:) ; 

vdot = a;

theta = primal.states(3,:) ; 

% vdot = a.*sin(theta); % FOR v_V TESTING

if const == 2
Qdot = heating_rate;
end

mfueldot = -Fueldt ; 

% theta  = primal.controls(1,:); %

thetadot  = primal.controls(1,:); %

% %=========================================================================================

Vdot = v.*sin(theta);

% Vdot = v; % FOR v_V TESTING


% HScaleddot = vScaled.*cos(theta)/ScaleH * Scalev;

% derivative of dynamic pressure
if const == 4
    
qdot = spline(drho(:,1), drho(:,2), V).* Vdot .* v.^2 + 2*rho.*v.*vdot;
end

%======================================================
if const == 1 || const == 3 || const == 5
XDOT = [Vdot;vdot; thetadot; mfueldot];
end

if const == 2
XDOT = [Vdot;vdot; thetadot; mfueldot; Qdot];
end

if const == 4
XDOT = [Vdot;vdot; thetadot; mfueldot; qdot];
end
%======================================================