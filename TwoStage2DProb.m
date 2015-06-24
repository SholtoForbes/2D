%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Scramjet Flight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;		
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global v

% Inputs ============================================
global MultiStage
MultiStage = 0; % 0 for no multistage 1 for multi

%=============================================== 


V0 = 0.; % Keep initial values zero
Vf = 10000.;

H0 = 0.;
Hf = 10000.;



%===================
% Problem variables:
% control factor: omega
% variable trajectory
%===================


% Scaling ========================================
global Scale
Scale =  Hf / 5; %  both are divided by this

HfScaled = Hf / Scale;
VfScaled = Vf / Scale;

%========================================================

%---------------------------------------
% bound and scale the state and control variables
%---------------------------------------

VL = -1.;
VU = 2*VfScaled;

HL = -1.;
HU = 2*HfScaled;


bounds.lower.states = [VL ; HL];
bounds.upper.states = [VU ; HU];


% control bounds
thetaL = -1.;
thetaU = 1.5;

bounds.lower.controls = [thetaL];
bounds.upper.controls = [thetaU]; 


%------------------
% bound the horizon
%------------------
% time bounds, this is SCALED
t0	    = 0;
tfMax 	= 20;   % swag for max tf; DO NOT set to Inf even for time-free problems
% remember to set higher than Vmax bounds min time

% MULTI STAGE
 
bounds.lower.time 	= [t0; t0];				
bounds.upper.time	= [t0; tfMax];


%-------------------------------------------
% Set up the bounds on the endpoint function
%-------------------------------------------
% See events file for definition of events function

bounds.lower.events = [V0;  VfScaled; H0; HfScaled];


% bounds.lower.events = [V0;  VfScaled; H0; HfScaled];


bounds.upper.events = bounds.lower.events;      % equality event function bounds



% PATH BOUNDS IF NECESSARY


%============================================
% Define the problem using DIDO expresssions:
%============================================
Brac_1.cost 		= 'TwoStage2DCost';
Brac_1.dynamics	    = 'TwoStage2DDynamics';
Brac_1.events		= 'TwoStage2DEvents';		
%Path file optional	

Brac_1.bounds       = bounds;


% Node Definition ====================================================


algorithm.nodes		= [80];	


global nodes
nodes = algorithm.nodes;


%  Guess =================================================================

%  Guess =================================================================

% tfGuess = tfMax;
tfGuess = 9.7; % this needs to be close to make sure solution stays withing Out_Force bounds


guess.states(1,:) = [0, VfScaled]; %v
guess.states(2,:) = [0, HfScaled]; %H

guess.controls(1,:)    = [0.78,0.78]; %a

guess.time        = [t0, tfGuess];




% Tell DIDO the guess.  Note: The guess-free option is not available when
% using "knots"
%========================
algorithm.guess = guess;
% algorithm.guess = primal_old;
% %========================
%=====================================================================================


% Call dido
% =====================================================================
tStart= cputime;    % start CPU clock 
[cost, primal, dual] = dido(Brac_1, algorithm);
runTime = cputime-tStart
% ===================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          OUTPUT             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global dfuel
dfuel

global StartingV
V = primal.states(1,:)*Scale + StartingV; 

H = primal.states(2,:)*Scale ; 

t = primal.nodes;

theta = primal.controls(1,:);


%calculating for interest
% c = 300.; % this will need to be brought into line with vehicle model

% M = v./ScaleFactor/c;
global M
global v_array
v_array
global m

figure(1)

subplot(3,4,2)
plot(H, V)
title('Trajectory')

subplot(3,4,4)
plot(t, v)
title('v')


subplot(3,4,5)
plot(t(1:end-1), M)
title('M')

subplot(3,4,7)
plot(t, theta)
title('theta')

subplot(3,4,8)
plot(t(1:end-1), m)
title('mass')


lam1 = dual.dynamics(1,:);

subplot(3,4,9);
plot(t, lam1);
title('costates')
xlabel('time');
ylabel('costates');
legend('\lambda_1');

subplot(3,4,10)
H = dual.Hamiltonian(1,:);
plot(t,H);
title('Hamiltonian')


% 
% %------ Forward Simulation -----------
% 
% % need to replace this with CADAC
% 
% % Import Controls, Time and Initial States
% % these are the only things carried over from the PS method
% tau_Forward = tau;
% Mc_Forward = Mc;
% t_Forward = t;
% h_Forward(1) = h(1);
% v_Forward(1) = v(1);
% vh_Forward(1) = vh(1);
% vv_Forward(1) = vv(1);
% omega_Forward(1) = omega(1);
% theta_Forward(1) = theta(1);
% %--------------


% Iy_Forward = 1.;
% m_Forward = 1000.;
% 
% Fx_Forward = 0.; %Taking these out for testing
% Fz_Forward = 0.;
% My_Forward = 0.;




% simple forward method
% for i=2:length(Mc_Forward)
% omegadot_Forward(i-1) = (My_Forward  + Mc_Forward(i-1))/Iy_Forward;
% omega_Forward(i) = omegadot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + omega_Forward(i-1);
% 
% thetadot_Forward(i-1) = omega_Forward(i-1);
% theta_Forward(i) = thetadot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + theta_Forward(i-1);
% 
%     
% vhdot_Forward(i-1) = (Fx_Forward.*cos(theta_Forward(i-1)) + Fz_Forward.*sin(theta_Forward(i-1))  + tau_Forward(i-1).*cos(theta_Forward(i-1)))/m_Forward;
% vh_Forward(i) = vhdot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + vh_Forward(i-1);
% 
% vvdot_Forward(i-1) = (-Fx_Forward.*sin(theta_Forward(i-1)) + Fz_Forward.*cos(theta_Forward(i-1))  + tau_Forward(i-1).*sin(theta_Forward(i-1)))/m_Forward;
% vv_Forward(i) = vvdot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + vv_Forward(i-1);
% 
% 
% hdot_Forward(i-1) = vh_Forward(i-1);
% h_Forward(i) = hdot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + h_Forward(i-1);
% 
% vdot_Forward(i-1) = vv_Forward(i-1);
% v_Forward(i) = vdot_Forward(i-1)*(t_Forward(i) - t_Forward(i-1)) + v_Forward(i-1);
% 
% end
% 
% 
% figure(2)
% plot(h_Forward, v_Forward)






primal_old = primal;




