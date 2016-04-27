function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dynamics for the Brac:1 Formulation of the Brachistochrone Prob
% Template for A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

V = primal.states(1,:);	    
v = primal.states(2,:);		
theta = primal.states(3,:);		
mfuel = primal.states(4,:);		

thetadot  = primal.controls;

%=======================================================
% Equations of Motion:
%=======================================================
Vdot = v.*sin(theta);
						

T = 10; %Thrust
m = 1 + mfuel;
vdot = T./m - 1*sin(theta)./m;   % 'faux gravity' 

mfueldot = -1*ones(1,length(primal.nodes));% mass flow rate
%======================================================
XDOT = [Vdot; vdot; thetadot; mfueldot];
%======================================================