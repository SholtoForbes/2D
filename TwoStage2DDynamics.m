function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics

% This uses velocity calculated in the Cost file

% This file is calculated after the Cost file in the iterative process 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% global v
global a
global Fueldt
global heating_rate

%changed notation to horizontal and vertical, x and y in plane of vehicle
V = primal.states(1,:) ; 

v = primal.states(2,:) ; 

vdot = a;

theta = primal.states(3,:) ; 

Qdot = heating_rate;


mfueldot = -Fueldt ; 

% theta  = primal.controls(1,:); %

thetadot  = primal.controls(1,:); %

% %=========================================================================================

Vdot = v.*sin(theta);
% HScaleddot = vScaled.*cos(theta)/ScaleH * Scalev;

% %====================================================================


%======================================================

% XDOT = [Vdot;vdot; thetadot; mfueldot];
XDOT = [Vdot;vdot; thetadot; mfueldot; Qdot];
%======================================================