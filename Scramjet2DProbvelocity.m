%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Scramjet Flight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;		
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inputs ============================================

%===============================================


%===================
% Problem variables:

%===================
global H_input
H_input = [0,100,200,300,400,500,600,700,800,900,1000] % Flight track starting at 0,0
global V_input
V_input = [0,100,200,300,400,500,600,700,800,900,1000]


HL = H_input(1);
HU = H_input(end);

global theta_array
theta_array = atan(V_input(2:end)./H_input(2:end)); %this is calculated from 2nd term onwards as first term will always give NaN (also used in dynamics file)

global StartingV
StartingV = 20000; %Initial altitude in m

%---------------------------------------
% bounds the state and control variables
%---------------------------------------

% vHL = 1000.; vHU = 10000.;  % Box Constraints. These are important, setting upper box constraint equal to upper bounds on path does not work, nor does setting this too high
% vVL = 1000.; vVU = 10000.;  % Keep these in terms of scaled h and v

vL = 2000.; vU = 3000.;

HL_box = 2*HL;
HU_box = 2*HU;

% fuel, bounds are arbitrary
fuelL = 0;
fuelU = 100;


bounds.lower.states = [vL ; fuelL; HL_box];
bounds.upper.states = [vU;  fuelU; HU_box];


bounds.lower.controls = [-20000.];
bounds.upper.controls = [20000.]; % Control bounds



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

% bounds.lower.events = [1.; 1.; HL; 1. ; 1.; HU]; %with velocity
% constraints
bounds.lower.events = [HL;  HU; 2500];

bounds.upper.events = bounds.lower.events;      % equality event function bounds

%============================================
% Define the problem using DIDO expresssions:
%============================================
Brac_1.cost 		= 'Scramjet2DCostvelocity';
Brac_1.dynamics	    = 'Scramjet2DDynamicsvelocity';
Brac_1.events		= 'Scramjet2DEventsvelocity';		
%Path file optional	

Brac_1.bounds       = bounds;
%====================================================

% Dont know how this changes the output yet...
algorithm.nodes		= [70];					    % represents some measure of desired solution accuracy

% algorith.mode = 'accurate';  %this did not seem to make a difference 28/4


%  Guess =================================================================
guess.states(1,:) = [2500, 2500, 2500]; %v
guess.states(2,:) = [50, 50,50]; %fuel
guess.states(3,:) = [0, 500, 1000]; %H

guess.controls(1,:)    = [100,100, 100]; %a, these are net force so 0 guess

guess.time        = [t0,500/2500, 1000/2500];


% Tell DIDO the guess.  Note: The guess-free option is not available when
% using "knots"
%========================
algorithm.guess = guess;
% algorithm.guess = primal_old;
%========================


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




% vH = primal.states(1,:);
% vV = primal.states(2,:);
v = primal.states(1,:) ; 

fuel = primal.states(2,:);

H = primal.states(3,:);

t = primal.nodes;

a = primal.controls(1,:);


%calculating for interest
c = 300.; % this will need to be brought into line with vehicle model
% M = sqrt((vH).^2 + (vV).^2)/c ;
M = v./c;


figure(1)

subplot(3,4,1)
plot(t, v)
title('v')

% subplot(3,4,2)
% plot(t, vV)
% title('vv')

subplot(3,4,3)
plot(t, fuel)
title('fuel')

subplot(3,4,4)
plot(t, H)
title('H')

subplot(3,4,5)
plot(t, M)
title('M')

subplot(3,4,7)
plot(t, a)
title('a')


% lam1 = dual.dynamics(1,:);
% lam2 = dual.dynamics(2,:);
% lam3 = dual.dynamics(3,:);
% lam4 = dual.dynamics(4,:);
% 
% subplot(3,4,9);
% plot(t, [lam1; lam2; lam3; lam4]);
% title('costates')
% xlabel('time');
% ylabel('costates');
% legend('\lambda_1', '\lambda_2', '\lambda_3', '\lambda_4');
% 
% subplot(3,4,10)
% H = dual.Hamiltonian(1,:);
% plot(t,H);
% title('Hamiltonian')

lam1 = dual.dynamics(1,:);
lam2 = dual.dynamics(2,:);
lam3 = dual.dynamics(3,:);

subplot(3,4,9);
plot(t, [lam1; lam2; lam3]);
title('costates')
xlabel('time');
ylabel('costates');
legend('\lambda_1', '\lambda_2', '\lambda_3');

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




