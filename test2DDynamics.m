function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = primal.states(1,:) ;  
y = primal.states(2,:);		
vx = primal.states(3,:);		
vy = primal.states(4,:);

theta  = primal.controls(1,:);
tau  = primal.controls(2,:);

%=======================================================
% Equations of Motion:
%=======================================================

%neglecting AoA, fixed reference frame
m = 10;
Cl = 1;
Cd = 1;
rho = 1;
A = 3;


% Original
% xdot = vx;
% ydot = vy;							 
% vxdot = m*(T.*cos(theta) - 0.5*rho*A*Cl*(vx.^2 + vy.^2).*sin(theta) - 0.5*rho*A*Cd*(vx.^2 + vy.^2).*cos(theta));  
% vydot = m*(T.*sin(theta) + 0.5*rho*A*Cl*(vx.^2 + vy.^2).*cos(theta) - 0.5*rho*A*Cd*(vx.^2 + vy.^2).*sin(theta) - 9.8);  


% scaling test (all other scaling variables currently =1) I need to analyse
% this better, mine is much more complex than brac problem
% need to change all scaling variables in this and problem file together
X = 10;
Y = 10;
VX = 1;
VY = 1;
TAU = 1;
THETA = 1;
T = 10;

xdot = vx*VX*T/X;
ydot = vy*VY*T/Y;							 
vxdot = T/VX*m*(tau*TAU.*cos(theta*THETA) - 0.5*rho*A*Cl*((vx*VX).^2 + (vy*VY).^2).*sin(theta*THETA) - 0.5*rho*A*Cd*((vx*VX).^2 + (vy*VY).^2).*cos(theta*THETA));  
vydot = T/VY*m*(tau*TAU.*sin(theta*THETA) + 0.5*rho*A*Cl*((vx*VX).^2 + (vy*VY).^2).*cos(theta*THETA) - 0.5*rho*A*Cd*((vx*VX).^2 + (vy*VY).^2).*sin(theta*THETA) - 9.8);  

%======================================================
XDOT = [xdot; ydot; vxdot; vydot];
%======================================================