%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Scramjet Flight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;		
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%velocity primal
% global v

% Inputs ============================================
global communicator
% communicator = importdata('communicator.txt');
communicator = importdata('communicator_extrapolate.txt');
global communicator_trim
% communicator_trim = importdata('communicator_trim.txt');
communicator_trim = importdata('communicator_trim_extrapolate.txt');


%=============================================== 
%Second Stage
V0 = 15000.; % Keep initial values zero
Vf = 40000.; % Final values here are for guess and bounds, need to be fairly accurate

H0 = 0.;
Hf = 700000.;

v0 = 1864.13; % 50kpa q at 27000m
vf = 2979.83; % 50kpa q at 33000m

%dawids results have around 1 degree or under flight path angle
%===================
% Problem variables:
% control factor: omega
% variable trajectory
%===================

% Scaling ========================================

% How can I scale so that H and V are scaled differently... Maybe need to
% scale velocity separately
global ScaleH
% ScaleH =  Hf / 700000; %  Horizontal Scale
ScaleH = 1.;
global ScaleV
% ScaleV =  Vf / 6000; %  Vertical Scale
ScaleV = 1.;

global Scalev
Scalev = 1.;

HfScaled = Hf / ScaleH;
VfScaled = Vf / ScaleV;
v0Scaled = v0 / Scalev;
vfScaled = vf / Scalev;


global ThetaScale
ThetaScale = 1.;
%========================================================

%---------------------------------------
% bound and scale the state and control variables
%---------------------------------------

VL = -1.;
VU = 1.0*VfScaled; % makes this exactly 40000

HL = -1.;
HU = 1.2*HfScaled;

vL = 1500/Scalev;
vU = 3100/Scalev; % This limit must not cause the drag force to exceed the potential thrust of the vehicle, otherwise DIDO will not solve

% bounds.lower.states = [VL ; HL; vL];
% bounds.upper.states = [VU ; HU; vU];

thetaL = -.2; %  NEED TO WATCH THAT THIS IS NOT OVERCONSTRAINING
thetaU = 0.5;

bounds.lower.states = [VL ; HL; vL; thetaL];
bounds.upper.states = [VU ; HU; vU; thetaU];

% control bounds
% thetaL = -1.;
% thetaU = 1.5;

% thetaL = -.2; %  NEED TO WATCH THAT THIS IS NOT OVERCONSTRAINING
% thetaU = .3;

thetadotL = -0.15;
thetadotU = 0.15;

bounds.lower.controls = [thetadotL];
bounds.upper.controls = [thetadotU]; 


%------------------
% bound the horizon
%------------------
% time bounds, this is unscaled

t0	    = 0;
tfMax 	= Hf/1500;   %  max tf; DO NOT set to Inf even for time-free problems % remember to set higher than Vmax bounds min time

bounds.lower.time 	= [t0; t0];				
bounds.upper.time	= [t0; tfMax];




%-------------------------------------------
% Set up the bounds on the endpoint function
%-------------------------------------------
% See events file for definition of events function


% bounds.lower.events = [V0;   H0; v0Scaled; vfScaled; 0];

% bounds.lower.events = [V0;   H0; v0Scaled; vfScaled];

bounds.lower.events = [H0; v0Scaled; vfScaled];

% bounds.lower.events = [v0Scaled; vfScaled];

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

tfGuess = tfMax; % this needs to be close to make sure solution stays withing Out_Force bounds


guess.states(1,:) = [0 ,VfScaled]; %v
guess.states(2,:) = [0,HfScaled]; %H

guess.states(3,:) = [v0, vf]; %H
guess.states(4,:) = [atan((Vf-V0)/(Hf-H0)),atan((Vf-V0)/(Hf-H0))]*ThetaScale; 

guess.controls(1,:)    = [0,0]; 
% guess.controls(1,:)    = [atan((Vf-V0)/(Hf-H0)),atan((Vf-V0)/(Hf-H0))]*ThetaScale; 

guess.time        = [t0 ,tfGuess];



% Tell DIDO the guess.  Note: The guess-free option is not available when
% using "knots"
%========================
algorithm.guess = guess;
% algorithm.guess = primal_old;
% %========================
% algorithm.mode = 'accurate';
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
StartingV = 0;
V = primal.states(1,:)*ScaleV + StartingV; 

H = primal.states(2,:)*ScaleH ; 

%velocity primal
v = primal.states(3,:)*Scalev;

t = primal.nodes;

% theta = primal.controls(1,:);

theta = primal.states(4,:);
thetadot = primal.controls(1,:);


%calculating for interest
% c = 300.; % this will need to be brought into line with vehicle model

% M = v./ScaleFactor/c;
global M
% global v_array
% v_array
global m
global q
global Fd
global Fueldt
global Endcost
global Thrust

dt = t(2:end)-t(1:end-1); % Time change between each node pt
FuelUsed = zeros(1,nodes-1);
FuelUsed(1) = dt(1)*Fueldt(1);
for i = 2:nodes-1
    FuelUsed(i) = dt(i).*Fueldt(i) + FuelUsed(i-1);
end

figure(1)

subplot(4,4,[1,4])
hold on
plot(H, V)
plot(H(algorithm.nodes(1)), V(algorithm.nodes(1)), '+', 'MarkerSize', 10, 'MarkerEdgeColor','r')
title('Trajectory')

subplot(4,4,5)
hold on
plot(t, v)
plot(t(algorithm.nodes(1)), v(algorithm.nodes(1)), '+', 'MarkerSize', 10, 'MarkerEdgeColor','r')
title('v')


subplot(4,4,6)
plot(t, M)
title('M')

subplot(4,4,7)
plot(t, q)
title('q')

subplot(4,4,8)
hold on
plot(t, rad2deg(theta))
plot(t(algorithm.nodes(1)), rad2deg(theta(algorithm.nodes(1))), '+', 'MarkerSize', 10, 'MarkerEdgeColor','r')
title('theta (Deg)')

subplot(4,4,13)
hold on
plot(t, rad2deg(thetadot))
title('thetadot (Deg/s)')

subplot(4,4,9)
plot(t, m)
title('mass')

subplot(4,4,10)
plot(t, Fd)
title('Drag Force')

subplot(4,4,14)
hold on
plot(t(1:end-1), FuelUsed)
bar(t(end), -Endcost)
title('Fuel and End Cost')

subplot(4,4,11);
plot(t, dual.dynamics);
title('costates')
xlabel('time');
ylabel('costates');
legend('\lambda_1', '\lambda_2', '\lambda_3');

subplot(4,4,12)
H = dual.Hamiltonian(1,:);
plot(t,H);
title('Hamiltonian')

subplot(4,4,15)
plot(t, Thrust)
title('Thrust (N)')

Isp = Thrust./Fueldt;

subplot(4,4,16)
plot(t, Isp)
title('Isp')


% 
% %------ Forward Simulation -----------
% 
% % potentially need to replace this with CADAC
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




