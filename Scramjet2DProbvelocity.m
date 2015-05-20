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
V_input = [0,50,100,150,200,250,300,350,400,450,500]
% V_input = [0,0,0,0,0,0,0,0,0,0,0]

global theta_array
theta_array = atan((V_input(2:end)-V_input(1:end-1))./(H_input(2:end)-H_input(1:end-1))) 

% Set required initial values
global StartingV
StartingV = 20000; %Initial altitude in m

global theta_initial
theta_initial = 0.78;

global v_initial
v_initial = 2500;

% Scaling ========================================
H_Scaled = 1000;

global ScaleFactor
ScaleFactor = H_Scaled/H_input(end);
%========================================================

%---------------------------------------
% bound and scale the state and control variables
%---------------------------------------

HL = H_input(1).*ScaleFactor;
HU = H_input(end).*ScaleFactor;

vL = 1000.*ScaleFactor;
vU = 10000.*ScaleFactor;

HL_box = 2*HL;
HU_box = 2*HU;

% fuel, bounds are arbitrary
fuelL = 0;
fuelU = 100;


bounds.lower.states = [vL ; fuelL; HL_box];
bounds.upper.states = [vU;  fuelU; HU_box];


% control (acceleration) bounds
aL = -150.*ScaleFactor;
aU = 150.*ScaleFactor;

bounds.lower.controls = [aL];
bounds.upper.controls = [aU]; 


%------------------
% bound the horizon
%------------------
% time bounds, this is SCALED
t0	    = 0;
tfMax 	= 15.;   % swag for max tf; DO NOT set to Inf even for time-free problems
% remember to set higher than Vmax bounds min time

bounds.lower.time 	= [t0; t0];				
bounds.upper.time	= [t0; tfMax];			    % Fixed time at t0 and a possibly free time at tf


%-------------------------------------------
% Set up the bounds on the endpoint function
%-------------------------------------------
% See events file for definition of events function

fuel_initial = 50;

bounds.lower.events = [HL;  HU; v_initial; fuel_initial];

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
guess.states(1,:) = [v_initial, v_initial, v_initial, v_initial, v_initial, v_initial, v_initial, v_initial, v_initial, v_initial, v_initial]*ScaleFactor; %v
guess.states(2,:) = [50, 50, 50 , 50, 50, 50, 50, 50, 50, 50, 50]; %fuel
guess.states(3,:) = [0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]*ScaleFactor; %H

guess.controls(1,:)    = [0,0, 0,0,0,0,0,0,0,0,0]*ScaleFactor; %a

guess.time        = [t0, 100/v_initial, 200/v_initial, 300/v_initial, 400/v_initial, 500/v_initial, 600/v_initial, 700/v_initial, 800/v_initial, 900/v_initial, 1000/v_initial]*ScaleFactor;


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




