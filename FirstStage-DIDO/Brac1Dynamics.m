function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global hscale
global mscale
global vscale
global Tscale

h = primal.states(1,:)*hscale;	    
v = primal.states(2,:)*vscale;		
gamma = primal.states(3,:);		
m = primal.states(4,:)*mscale;	

z = [h; v; gamma; m];


T  = primal.controls*Tscale;

phase = 'postpitch';
dz = rocketDynamics(z,T,phase);
rdot = dz(1,:);
vdot = dz(2,:);
gammadot = dz(3,:);



mdot = dz(4,:);

%======================================================
XDOT = [rdot/hscale; vdot/vscale; gammadot; mdot/mscale];
%======================================================