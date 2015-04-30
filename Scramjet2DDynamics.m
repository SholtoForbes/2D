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

tau  = primal.controls(1,:); %Thrust
Mc = primal.controls(2,:); %Control Moment



% theta  = primal.controls(1,:);
% tau  = primal.controls(2,:);

%=======================================================
% Equations of Motion:
%=======================================================

%neglecting AoA, fixed reference frame
m = 1000.;
Cl = .1;
Cd = .1;
rho = 0.02;
% rho = 1.;
A = 3.;

%======================================================


% Original
% xdot = vx;
% ydot = vy;							 
% vxdot = m*(T.*cos(theta) - 0.5*rho*A*Cl*(vx.^2 + vy.^2).*sin(theta) - 0.5*rho*A*Cd*(vx.^2 + vy.^2).*cos(theta));  
% vydot = m*(T.*sin(theta) + 0.5*rho*A*Cl*(vx.^2 + vy.^2).*cos(theta) - 0.5*rho*A*Cd*(vx.^2 + vy.^2).*sin(theta) - 9.8);  


% scaling test (all other scaling variables currently =1) I need to analyse
% this better, mine is much more complex than brac problem
% need to change all scaling variables in this and problem file together
% X = 10;
% Y = 10;
global X;
global Y;
VX = 1.;
VY = 1.;
TAU = 1.;
THETA = 1.;
T = 1.;

hdot = vh*VX*T/X;
vdot = vv*VY*T/Y;							 
% vxdot = T/VX/m*(tau*TAU.*cos(theta*THETA) - 0.5*rho*A*Cl*((vx*VX).^2 + (vy*VY).^2).*sin(theta*THETA) - 0.5*rho*A*Cd*((vx*VX).^2 + (vy*VY).^2).*cos(theta*THETA));  
% vydot = T/VY/m*(tau*TAU.*sin(theta*THETA) + 0.5*rho*A*Cl*((vx*VX).^2 + (vy*VY).^2).*cos(theta*THETA) - 0.5*rho*A*Cd*((vx*VX).^2 + (vy*VY).^2).*sin(theta*THETA) - 9.8);  


%===========================================================
% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments
Iy = 1.;

% Fx = -sqrt(vh.^2 + vv.^2); %these will need to come from the communicator matrix, for now just approximated a function that looks close to com mat values
% Fz = 2*sqrt(vh.^2 + vv.^2);
% My = 250*sqrt(vh.^2 + vv.^2);


% import data from force matrix
Out_force = dlmread('out_force.txt');



Fx = -1.1.*vh; 
% Fx = 0.; 
Fz = 0.;
My = 0.;


vhdot = (Fx.*cos(theta) + Fz.*sin(theta)  + tau.*cos(theta))/m;
vvdot = (-Fx.*sin(theta) + Fz.*cos(theta)  + tau.*sin(theta))/m;
thetadot = omega;
omegadot = (My  + Mc)/Iy;
%====================================================================



%======================================================
XDOT = [hdot; vdot; vhdot; vvdot; thetadot; omegadot];
%======================================================