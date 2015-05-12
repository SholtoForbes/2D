function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% need to add AoA

global HScale
global VScale


%changed notation to horizontal and vertical, x and y in plane of vehicle
h = primal.states(1,:) ;  
y = primal.states(2,:);		
vh = primal.states(3,:);		
vv = primal.states(4,:);
theta  = primal.states(5,:);
omega = primal.states(6,:);


% tau  = primal.controls(1,:); %Thrust
Fx  = primal.controls(1,:); %Thrust
Fz  = primal.controls(2,:); %Thrust
Mc = primal.controls(3,:); %Control Moment

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

hdot = vh;
vdot = vv;							 

%===========================================================

% Mach no
c = 1000; % speed of sound (replace with atmosphere)
M = sqrt((vh./HScale).^2 + (vv./VScale).^2)/c ;




% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments

Iy = 1.; %%%% CHANGE THIS

% import data from force matrix
Out_force = dlmread('out_force.txt');
%an initial interpolator for the force values at a fixed Arot, alpha and
%dynamic pressure (0,  -0.0174532925199 (negative up) , 45000.0)

% 1
% M_array = [4.5 , 5. , 5.5]; 
% Fx_array = [-36427.6593981 , -42995.3909773 , -50209.1507264];
% Fz_array = [ 26851.676829 , 25865.7310572 , 24420.6025981 ];
% My_array = [305002.235162 , 256125.242712 , 196654.950117 ];
% 
% % Scaled with a dot product of the scaling factors and the force directions
% 
% Fx = spline(M_array, Fx_array, M)  ;
% Fz = spline(M_array, Fz_array, M)  ;
% Fz = 0;


% Fx = -sqrt(vh.^2 + vv.^2) .* (HScale.*cos(theta) + VScale.*sin(theta));
% % Fx = -50000.;
% Fz = -0.1*sqrt(vh.^2 + vv.^2).* (HScale.*sin(theta) + VScale.*cos(theta)); 
% Fz = 0.;
My = 0.; %moment



% 1
% vhdot = (Fx.*cos(theta) - Fz.*sin(theta)  + tau.*cos(theta))/m .* HScale;
% vvdot = ((Fx.*sin(theta) + Fz.*cos(theta)  + tau.*sin(theta))/m - g) .* VScale;


vhdot = (Fx.*cos(theta) - Fz.*sin(theta)  )/m .* HScale;
vvdot = ((Fx.*sin(theta) + Fz.*cos(theta) )/m - g) .* VScale;

thetadot = omega;
omegadot = (My  + Mc)/Iy;
%====================================================================


%======================================================
XDOT = [hdot; vdot; vhdot; vvdot; thetadot; omegadot];
%======================================================