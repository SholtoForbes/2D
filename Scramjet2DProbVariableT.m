%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Scramjet Flight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;		
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inputs ============================================
global MultiStage
MultiStage = 0;

%===============================================


%===================
% Problem variables:

%===================
global H_input
H_input = [0,10000,20000,30000,40000,50000,60000,70000,80000,90000,100000] % Flight track starting at 0,0, 100km flight distance

% Vertical track input
global V_input
% V_input = [0,0,0,0,0,0,0,0,0,0,0] % Horizontal flight
% V_input = [0,1000,2000,3000,4000,5000,6000,7000,8000,9000,10000] % 10km altitude increase
V_input = 10*(sqrt(H_input+10) - sqrt(10)) % a square root curve


global theta_array
theta_array = atan((V_input(2:end)-V_input(1:end-1))./(H_input(2:end)-H_input(1:end-1))) 

% Set required initial values
global StartingV
StartingV = 20000; %Initial altitude in m

global theta_initial
theta_initial = 0.;



% Scaling ========================================
H_Scaled = 100;

global ScaleFactor
ScaleFactor = H_Scaled/H_input(end);
%========================================================


global v_initial
v_initial = 2500.*ScaleFactor;

v_final = 2500.*ScaleFactor;

%---------------------------------------
% bound and scale the state and control variables
%---------------------------------------

HL = H_input(1).*ScaleFactor;
HU = H_input(end).*ScaleFactor;

% vL = 1500.*ScaleFactor;
vL = 0.;
vU = 5000.*ScaleFactor;

HL_box = 2*HL;
HU_box = 2*HU;

% fuel, bounds are arbitrary

bounds.lower.states = [vL ;  HL_box];
bounds.upper.states = [vU;  HU_box];


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
tfMax 	= HU_box/vL;   % swag for max tf; DO NOT set to Inf even for time-free problems
% tfMax = 15
% remember to set higher than Vmax bounds min time

% bounds.lower.time 	= [t0; t0];				
% bounds.upper.time	= [t0; tfMax];			    % Fixed time at t0 and a possibly free time at tf

% MULTI STAGE
if MultiStage == 1
    bounds.lower.time 	= [t0 t0 t0];				
    bounds.upper.time	= [t0 tfMax/2 tfMax];			    % Fixed time at t0 and a possibly free time at tf
else  
    bounds.lower.time 	= [t0; t0];				
    bounds.upper.time	= [t0; tfMax];
end

%-------------------------------------------
% Set up the bounds on the endpoint function
%-------------------------------------------
% See events file for definition of events function

% bounds.lower.events = [HL;  HU; v_initial; v_final];
% bounds.upper.events = bounds.lower.events;      % equality event function bounds

%MULTI STAGE
if MultiStage == 1
    dv = 0.;
    dH = 0.;
    bounds.lower.events = [HL;  HU; v_initial; v_final; dv; dH];
    bounds.upper.events = bounds.lower.events;      % equality event function bounds
else
    bounds.lower.events = [HL;  HU; v_initial; v_final];
    bounds.upper.events = bounds.lower.events;      % equality event function bounds
end

% PATH BOUNDS IF NECESSARY


%============================================
% Define the problem using DIDO expresssions:
%============================================
Brac_1.cost 		= 'Scramjet2DCostVariableT';
Brac_1.dynamics	    = 'Scramjet2DDynamicsVariableT';
Brac_1.events		= 'Scramjet2DEventsVariableT';		
%Path file optional	

Brac_1.bounds       = bounds;



% MULTI STAGE ========================================================================
%  Tell DIDO that all your events are 'hard'; this redundant statement will
%  be removed in future versions of DIDO
%========================================================================
if MultiStage ==1
    algorithm.knots.definitions  = {'hard','hard','hard'};
end
%========================================================================
% For a 3 stage problem, you need to add another 'hard' etc. Yes, it's
% redundant.



% Node Definition ====================================================

if MultiStage ==1
    algorithm.nodes     = [60 60];
else
    algorithm.nodes		= [80];	
end



% %  Guess =================================================================
% This is also useable for multi stage! guess for stage transition not
% necessary 
guess.states(1,:) = [v_initial,  v_initial]*ScaleFactor; %v
guess.states(2,:) = [0,  1000]*ScaleFactor; %H

guess.controls(1,:)    = [0,0]*ScaleFactor; %a

guess.time        = [t0, 1000/v_initial]*ScaleFactor;

%  Guess MULTI STAGE =================================================================
% This guess includes a guess for stage transition, which seems to
% constrain the trajectory
% guess.states(1,:) = [v_initial,  v_initial,  v_initial]*ScaleFactor; %v
% guess.states(2,:) = [0, 500 , 1000]*ScaleFactor; %H
% 
% guess.controls(1,:)    = [0,0,0]*ScaleFactor; %a

tfGuess = 1000/v_initial*ScaleFactor
% guess.time        = [t0, tfGuess/2,  tfGuess];


% algorith.mode = 'accurate';  %this did not seem to make a difference 28/4


% Guess interior event time:
algorithm.knots.locations    = [t0, tfGuess/2,  tfGuess];


% Tell DIDO the guess.  Note: The guess-free option is not available when
% using "knots"
%========================
algorithm.guess = guess;
% algorithm.guess = primal_old;
%========================
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




% vH = primal.states(1,:);
% vV = primal.states(2,:);
v = primal.states(1,:) ; 

H = primal.states(2,:);

t = primal.nodes;

a = primal.controls(1,:);


%calculating for interest
% c = 300.; % this will need to be brought into line with vehicle model

% M = v./ScaleFactor/c;
global M


figure(1)

subplot(3,4,1)
plot(H_input, V_input)
title('Input Flight Track')

subplot(3,4,2)
plot(t, v)
title('v')


subplot(3,4,4)
plot(t, H)
title('H')

subplot(3,4,5)
plot(t, M)
title('M')

subplot(3,4,7)
plot(t, a)
title('a')


if MultiStage ==0
    lam1 = dual.dynamics(1,:);
    lam2 = dual.dynamics(2,:);

    subplot(3,4,9);
    plot(t, [lam1; lam2]);
    title('costates')
    xlabel('time');
    ylabel('costates');
    legend('\lambda_1', '\lambda_2');

    subplot(3,4,10)
    H = dual.Hamiltonian(1,:);
    plot(t,H);
    title('Hamiltonian')
end

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




