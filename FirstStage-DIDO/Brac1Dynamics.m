function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dynamics for the Brac:1 Formulation of the Brachistochrone Prob
% Template for A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

h = primal.states(1,:);	    
v = primal.states(2,:);		
gamma = primal.states(3,:);		
mfuel = primal.states(4,:);	

z = [h; v; gamma; mfuel];

T  = primal.controls;


[rdot,vdot,gammadot,mfueldot] = rocketDynamics(z,T);

%======================================================
XDOT = [rdot; vdot; gammadot; mfueldot];
%======================================================