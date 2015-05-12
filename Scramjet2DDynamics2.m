function XDOT = ScramDynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% need to add AoA

global HScale
global VScale


%changed notation to horizontal and vertical, x and y in plane of vehicle
H = primal.states(1,:) ;  
V = primal.states(2,:);		
vh = primal.states(3,:);		
vv = primal.states(4,:);


Fh  = primal.controls(1,:); %Thrust
Fv  = primal.controls(2,:); %Thrust


%=======================================================
% Equations of Motion:
%=======================================================

%neglecting AoA, fixed reference frame
m = 3000.;
Cl = .1;
Cd = .1;
rho = 0.02;
A = 3.;
g = 9.81;
%======================================================


%

Hdot = vh;
Vdot = vv;							 

vhdot = Fh;
vvdot = Fv;
%===========================================================

% Mach no
c = 1000; % speed of sound (replace with atmosphere)
M = sqrt((vh./HScale).^2 + (vv./VScale).^2)/c ;




% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments


%====================================================================


%======================================================
XDOT = [Hdot; Vdot; vhdot; vvdot;];
%======================================================