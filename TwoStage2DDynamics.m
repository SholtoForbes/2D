function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics

% This uses velocity calculated in the Cost file

% This file is calculated after the Cost file in the iterative process 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global ScaleH
global ScaleV
global Scalev
global ThetaScale
% global v
global a

%changed notation to horizontal and vertical, x and y in plane of vehicle
VScaled = primal.states(1,:) ; 
HScaled = primal.states(2,:) ; 

%velocity primal
vScaled = primal.states(3,:) ; 
vdot = a/Scalev;

theta  = primal.controls(1,:)/ThetaScale; %

% %=========================================================================================

VScaleddot = vScaled.*sin(theta)/ScaleV * Scalev;
HScaleddot = vScaled.*cos(theta)/ScaleH * Scalev;

% %====================================================================


%======================================================
XDOT = [VScaleddot; HScaleddot; vdot];
%======================================================