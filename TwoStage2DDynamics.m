function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics

% This uses velocity calculated in the Cost file

% This file is calculated after the Cost file in the iterative process 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global ScaleH
global ScaleV
global ThetaScale
global v

%changed notation to horizontal and vertical, x and y in plane of vehicle
VScaled = primal.states(1,:) ; 
HScaled = primal.states(2,:) ; 

theta  = primal.controls(1,:)/ThetaScale; %

% %=========================================================================================

VScaleddot = v.*sin(theta)/ScaleV;
HScaleddot = v.*cos(theta)/ScaleH;

% %====================================================================


%======================================================
XDOT = [VScaleddot; HScaleddot];
%======================================================