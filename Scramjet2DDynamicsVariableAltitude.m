function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global H_input
global V_input
global theta_array
global StartingV
global theta_initial
global ScaleFactor

%changed notation to horizontal and vertical, x and y in plane of vehicle
V = primal.states(1,:) ; 
H = primal.states(2,:) ; 

theta  = primal.controls(1,:); %

global v
v = 1;

%=========================================================================================
% Calculate Derivative Terms
Vdot = v.*sin(theta);
Hdot = v.*cos(theta);


%====================================================================


%======================================================
XDOT = [Vdot; Hdot];
%======================================================