%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;				% always a good idea to begin with this!

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%
mTotal = 15000;   %(kg)  %Total lift-off mass
mFuel = 0.8*mTotal;  %(kg)  %mass of the fuel
mEmpty = mTotal-mFuel;  %(kg)  %mass of the rocket (without fuel)
global Tmax
Tmax = 200000;    %(N)   %Maximum thrust

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Pre-Pitchover Simulation                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
h0_prepitch = 0;  %Rocket starts on the ground
v0_prepitch = 0;  %Rocket starts stationary
m0_prepitch = mTotal;  %Rocket starts full of fuel
gamma0_prepitch = deg2rad(90);

phase = 'prepitch';
tspan = [0 20];
prepitch0 = [h0_prepitch, v0_prepitch, gamma0_prepitch, m0_prepitch];
[t_prepitch, prepitch] = ode45(@(t,prepitch) rocketDynamics(prepitch,Tmax,phase), tspan, prepitch0);

% % FOR TESTING
phase = 'postpitch';
Tratio = 0.88;
tspan = [0 (prepitch(end,4)-mEmpty)/(Tratio*60)];
postpitch0 = [prepitch(end,1), prepitch(end,2), deg2rad(89), prepitch(end,4)];
[t_postpitch, postpitch] = ode45(@(t,postpitch) rocketDynamics(postpitch,Tratio*Tmax,phase), tspan, postpitch0);

prepitch
postpitch


%---------------------------------------
% bounds the state and control variables
%---------------------------------------
global hscale
% hscale = 1000;
hscale = 1;

global mscale
% mscale = 10000;
mscale = 1;

global vscale
% vscale = 1000;
vscale = 1;

hL = 1/hscale; hU = 90000/hscale;
vL = 1; vU = 5000/vscale;
gammaL = -.01; gammaU = deg2rad(90);
mL = (mTotal - mFuel)/mscale; mU = prepitch(end,4)/mscale;

bounds.lower.states = [hL; vL; gammaL; mL];
bounds.upper.states = [hU; vU; gammaU; mU];

bounds.lower.controls = [Tmax/2];
global Tscale
% Tscale = 10000;
Tscale = 1;
bounds.upper.controls = [Tmax/Tscale];


%------------------
% bound the horizon
%------------------
t0	    = 0;
tfMax 	= 1000;   % swag for max tf; DO NOT set to Inf even for time-free problems

bounds.lower.time 	= [t0; t0];				
bounds.upper.time	= [t0; tfMax];			    % Fixed time at t0 and a possibly free time at tf


%-------------------------------------------
% Set up the bounds on the endpoint function
%-------------------------------------------
% See events file for definition of events function

bounds.lower.events = [prepitch(end,1)/hscale; prepitch(end,2)/vscale; deg2rad(89); mU/mscale; 62000; 0.0];
bounds.upper.events = bounds.lower.events;      % equality event function bounds


%============================================
% Define the problem using DIDO expresssions:
%============================================
Brac_1.cost 		= 'Brac1Cost';
Brac_1.dynamics	    = 'Brac1Dynamics';
Brac_1.events		= 'Brac1Events';		
%Path file not required for this problem formulation;	

Brac_1.bounds       = bounds;
%====================================================

algorithm.nodes		= [61];					    % represents some measure of desired solution accuracy





%Guess
guess.states(1,:) = [prepitch(end,1), 64000]/hscale;
guess.states(2,:) = [prepitch(end,2), 4000]/vscale;
guess.states(3,:) = [deg2rad(89), 0];
guess.states(4,:) = [mU, mL]/mscale;

guess.controls(1,:)    = [.88*Tmax,.88*Tmax]/Tscale; 

guess.time        = [0 ,210];
algorithm.guess = guess;



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
gamma = primal.states(3,:);
m = primal.states(4,:);
T  = primal.controls;
t = primal.nodes;

%=============================================
figure;
plot(t, h, '-o', t, v, '-x', t, gamma, '-.' ,t ,m, '--');
title('Brachistochrone States: Brac 1')
xlabel('time');
ylabel('states');
legend('V', 'v', 'theta', 'mfuel');

figure;
plot(t,T)
xlabel('time');
ylabel('thrust');
%----------------------------------------------
figure;
plot(t, dual.Hamiltonian);
title('Brachistochrone Hamiltonian Evolution');
legend('H');
xlabel('time');
ylabel('Hamiltonian Value');

%----------------------------------------------
figure;
plot(t, dual.dynamics);
title('Brachistochrone Costates: Brac 1')
xlabel('time');
ylabel('costates');
legend('\lambda_x', '\lambda_y', '\lambda_v');
% ==============================================



% % TESTING OPTIMALITY
% 
% % GRADIENT NORMALITY CONDITION


% % Lagrangian of the Hamiltonian 
LH = dual.Hamiltonian + dot(dual.states,primal.states) + dual.controls.*primal.controls;

dLHdu = diff(LH)./diff(primal.controls);

