function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% need to add AoA

global HScale
global VScale
global StartingV 
global EndV 
global EndH

%changed notation to horizontal and vertical, x and y in plane of vehicle
H = primal.states(1,:) ;  
V = primal.states(2,:);	
theta  = primal.states(3,:);
omega = primal.states(4,:);
vh = primal.states(5,:);		
vv = primal.states(6,:);

fuel = primal.states(7,:); % might need to be changed to a generic efficiency term


% tau  = primal.controls(1,:); %Thrust

% Fx  = primal.controls(1,:); %Thrust
% Fz  = primal.controls(2,:); %Thrust
% Mc = primal.controls(3,:); %Control Moment


ax  = primal.controls(1,:); %Thrust
az  = primal.controls(2,:); %Thrust
omegadot = primal.controls(3,:); %Control Moment


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

						 

%===========================================================
% Vehicle Model for Fuel Term


% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments

Iy = 1.; %%%% CHANGE THIS

% import data from force matrix
Out_force = dlmread('out_force.txt');
% import data from atmosphere matrix
Atmosphere = dlmread('atmosphere.txt');


Vabs = V + StartingV;
c = spline( Atmosphere(:,1),  Atmosphere(:,5), Vabs);

% Mach no
% c = 300; % speed of sound (replace with atmosphere)
M = sqrt((vh./HScale).^2 + (vv./VScale).^2)./c ;



%an initial interpolator for the force values at a fixed Arot, alpha and
%dynamic pressure (0,  -0.0174532925199 (negative up) , 45000.0)

% 1

M_array = [4.5 , 5. , 5.5]; 
Fd_array = [-36427.6593981 , -42995.3909773 , -50209.1507264];
Flift_array = [ 26851.676829 , 25865.7310572 , 24420.6025981 ];
My_array = [305002.235162 , 256125.242712 , 196654.950117 ];



Fd = spline(M_array, Fd_array, M) ./ Vabs ; % drag force including fudge factor for altitude test!
% Fd = spline(M_array, Fd_array, M);
Flift = spline(M_array, Flift_array, M)  ;
My = spline(M_array, My_array, M)  ;



Thrust = ax*m - Fd + g*sin(theta);



% Thrust =   Fx - Fd;

% = Fz  - Flift; will need an additional z force term 







% 1
% vhdot = (Fx.*cos(theta) - Fz.*sin(theta)  + tau.*cos(theta))/m .* HScale;
% vvdot = ((Fx.*sin(theta) + Fz.*cos(theta)  + tau.*sin(theta))/m - g) .* VScale;


%=========================================================================================
% Calculate Derivative Terms
hdot = vh;
vdot = vv;	

% vhdot = (Fx.*cos(theta) - Fz.*sin(theta)  )/m .* HScale;
% vvdot = ((Fx.*sin(theta) + Fz.*cos(theta) )/m - g) .* VScale;

vhdot = ax.*cos(theta) - az.*sin(theta)  ;
vvdot = ax.*sin(theta) + az.*cos(theta) ;

thetadot = omega;
% omegadot = (Mc)/Iy;


fueldot = -0.01* Thrust; % this will need to be changed
%====================================================================


%======================================================
XDOT = [hdot; vdot; thetadot; omegadot; vhdot; vvdot;  fueldot];
%======================================================