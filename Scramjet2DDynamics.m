function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% need to add AoA


%changed notation to horizontal and vertical, x and y in plane of vehicle
h = primal.states(1,:) ;  
y = primal.states(2,:);		
vh = primal.states(3,:);		
vv = primal.states(4,:);
theta  = primal.states(5,:);
omega = primal.states(6,:);
M = primal.states(7,:);

tau  = primal.controls(1,:); %Thrust
Mc = primal.controls(2,:); %Control Moment

%=======================================================
% Equations of Motion:
%=======================================================

%neglecting AoA, fixed reference frame
m = 1000.;
Cl = .1;
Cd = .1;
rho = 0.02;
A = 3.;

%======================================================


%

hdot = vh;
vdot = vv;							 

%===========================================================
% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments
Iy = 1.;

% import data from force matrix
Out_force = dlmread('out_force.txt');
%an initial interpolator for the force values at a fixed Arot, alpha and
%dynamic pressure (0,  -0.0174532925199 (negative up) , 45000.0)

% M_array = [4.5 , 5. , 5.5]; 
% Fx_array = [-36427.6593981 , -42995.3909773 , -50209.1507264];
% Fz_array = [ 26851.676829 , 25865.7310572 , 24420.6025981 ];
% My_array = [305002.235162 , 256125.242712 , 196654.950117 ];
% 
% 
% global HScale;
% global VScale;
% Fx = spline(M_array, Fx_array, M) * HScale ;% These give NaN values if not within interpolation range
% Fz = spline(M_array, Fx_array, M) * VScale ;



Fx = -sqrt(vh.^2 + vv.^2); 
% Fx = -50000.;
Fz = -0.1*sqrt(vh.^2 + vv.^2); 
% Fz = 0.;
My = 0.; %moment




vhdot = (Fx.*cos(theta) - Fz.*sin(theta)  + tau.*cos(theta))/m;
vvdot = (Fx.*sin(theta) + Fz.*cos(theta)  + tau.*sin(theta))/m;
thetadot = omega;
omegadot = (My  + Mc)/Iy;
%====================================================================

%Placeholder
c = 10;
vh;
vhdot;
vv;
vvdot;
Mdot = (vh.*vhdot + vv.*vvdot)./(c.*sqrt(vh.^2 + vv.^2)); %Mach derivative calculated from base principles

%======================================================
XDOT = [hdot; vdot; vhdot; vvdot; thetadot; omegadot; Mdot];
%======================================================