%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Scramjet Flight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;		
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Scaling

%================================================
TrueDh = 10.; % True horizontal distance
TrueDv = 10.; % True vertical distance

global HScale
HScale = 10./TrueDh ; % Horizontal scale
global VScale
VScale = 10./TrueDv ; % Vertical scale
%===============================================


%===================
% Problem variables:
%-------------------
% states = x, y, vx, vy, theta, omega
% controls = tau, Mc
%===================


%---------------------------------------
% bounds the state and control variables
%---------------------------------------

hL = 0; hU = 20.;  % Box Constraints. These are important, setting upper box constraint equal to upper bounds on path does not work, nor does setting this too high
vL = 0; vU = 20.;  % Keep these in terms of scaled h and v

vhL = 0.; vhU = 3.;
vvL = 0.; vvU = 3.;



thetaL = 0; thetaU = 1.57; %these will need to be adjusted
omegaL = 0; omegaU = 1.;


bounds.lower.states = [hL; vL; vhL; vvL; thetaL; omegaL];
bounds.upper.states = [hU; vU; vhU; vvU; thetaU; omegaU];

%ADJUSTED FOR NORMALISATION
bounds.lower.controls = [0.; 0.];
bounds.upper.controls = [100000.; 100.]; 

%------------------
% bound the horizon
%------------------
t0	    = 0;
tfMax 	= 15.;   % swag for max tf; DO NOT set to Inf even for time-free problems
% remember to set higher than Vmax bounds min time

bounds.lower.time 	= [t0; t0];				
bounds.upper.time	= [t0; tfMax];			    % Fixed time at t0 and a possibly free time at tf


%-------------------------------------------
% Set up the bounds on the endpoint function
%-------------------------------------------
% See events file for definition of events function



%Scaling tests
% bounds.lower.events = [0; 0; 2; 2; 100/X; 100/Y; 2 ; 2]; %works with
% X,Y,T =100, or all 10
% bounds.lower.events = [0; 0; 10; 10; 100/X; 100/Y; 10 ; 10];
bounds.lower.events = [0; 0; 10.; 10.];

% bounds.lower.events = [0; 0; 1; 1; 10/X; 10/Y; 1 ; 1];
bounds.upper.events = bounds.lower.events;      % equality event function bounds
% bounds.upper.events = [0; 0; 2; 2; 10/X; 10/Y; 3 ; 3];

%============================================
% Define the problem using DIDO expresssions:
%============================================
Brac_1.cost 		= 'Scramjet2DCost';
Brac_1.dynamics	    = 'Scramjet2DDynamics';
Brac_1.events		= 'Scramjet2DEvents';		
%Path file optional	

Brac_1.bounds       = bounds;
%====================================================

% Dont know how this changes the output yet...
algorithm.nodes		= [40];					    % represents some measure of desired solution accuracy

% algorith.mode = 'accurate';  %this did not seem to make a difference 28/4


% Call dido
tStart= cputime;    % start CPU clock 
[cost, primal, dual] = dido(Brac_1, algorithm);
runTime = cputime-tStart
% Ta da!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          OUTPUT             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




h = primal.states(1,:);
v = primal.states(2,:);
vh = primal.states(3,:);
vv = primal.states(4,:);
theta = primal.states(5,:);
omega = primal.states(6,:);

t = primal.nodes;

tau = primal.controls(1,:);
Mc = primal.controls(2,:);


figure(1)
subplot(3,3,1)
plot(h,v)
title('h-v')
subplot(3,3,2)
plot(t, vh)
title('vh')
subplot(3,3,3)
plot(t, vv)
title('vv')

subplot(3,3,4)
plot(t, theta)
title('theta')
subplot(3,3,5)
plot(t, omega)
title('omega')
subplot(3,3,6)
plot(t, Mc)
title('Mc')
subplot(3,3,7)
plot(t, tau)
title('tau')

lam1 = dual.dynamics(1,:);
lam2 = dual.dynamics(2,:);
lam3 = dual.dynamics(3,:);
lam4 = dual.dynamics(4,:);

subplot(3,3,8);
plot(t, [lam1; lam2; lam3; lam4]);
title('costates')
xlabel('time');
ylabel('costates');
legend('\lambda_1', '\lambda_2', '\lambda_3', '\lambda_4');

subplot(3,3,9)
H = dual.Hamiltonian(1,:);
plot(t,H);
title('Hamiltonian')



%------ Forward Simulation -----------

% need to replace this with CADAC

% Import Controls, Time and Initial States
% these are the only things carried over from the PS method
tau_Forward = tau;
Mc_Forward = Mc;
t_Forward = t;
h_Forward(1) = h(1);
v_Forward(1) = v(1);
vh_Forward(1) = vh(1);
vv_Forward(1) = vv(1);
omega_Forward(1) = omega(1);
theta_Forward(1) = theta(1);
%--------------


Iy_Forward = 1.;
m_Forward = 1000.;

Fx_Forward = 0.; %Taking these out for testing
Fz_Forward = 0.;
My_Forward = 0.;




%simple forward method
for i=2:length(Mc_Forward)
omegadot_Forward(i-1) = (My_Forward  + Mc_Forward(i-1))/Iy_Forward;
omega_Forward(i) = omegadot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + omega_Forward(i-1);

thetadot_Forward(i-1) = omega_Forward(i-1);
theta_Forward(i) = thetadot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + theta_Forward(i-1);

    
vhdot_Forward(i-1) = (Fx_Forward.*cos(theta_Forward(i-1)) + Fz_Forward.*sin(theta_Forward(i-1))  + tau_Forward(i-1).*cos(theta_Forward(i-1)))/m_Forward;
vh_Forward(i) = vhdot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + vh_Forward(i-1);

vvdot_Forward(i-1) = (-Fx_Forward.*sin(theta_Forward(i-1)) + Fz_Forward.*cos(theta_Forward(i-1))  + tau_Forward(i-1).*sin(theta_Forward(i-1)))/m_Forward;
vv_Forward(i) = vvdot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + vv_Forward(i-1);


hdot_Forward(i-1) = vh_Forward(i-1);
h_Forward(i) = hdot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + h_Forward(i-1);

vdot_Forward(i-1) = vv_Forward(i-1);
v_Forward(i) = vdot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + v_Forward(i-1);

end


figure(2)
plot(h_Forward, v_Forward)











