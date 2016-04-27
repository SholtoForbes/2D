%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Problem (script) file for the Brachistochrone Problem Formulation, Brac: 1
% Template for A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;				% always a good idea to begin with this!

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%===================
% Problem variables:
%-------------------
% states = (x, y, v)
% controls = theta
%===================

%---------------------------------------
% bounds the state and control variables
%---------------------------------------


VL = 0; VU = 100;
vL = 0; vU = 30;
thetaL = 0; thetaU = deg2rad(90);
mfuelL = 0; mfuelU = 10;

bounds.lower.states = [VL; vL; thetaL; mfuelL];
bounds.upper.states = [VU; vU; thetaU; mfuelU];

bounds.lower.controls = [-pi];
bounds.upper.controls = [pi];


%------------------
% bound the horizon
%------------------
t0	    = 0;
tfMax 	= 10;   % swag for max tf; DO NOT set to Inf even for time-free problems

bounds.lower.time 	= [t0; t0];				
bounds.upper.time	= [t0; tfMax];			    % Fixed time at t0 and a possibly free time at tf


%-------------------------------------------
% Set up the bounds on the endpoint function
%-------------------------------------------
% See events file for definition of events function

bounds.lower.events = [0; 0; 5; 10; 0; 0];
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

algorithm.nodes		= [90];					    % represents some measure of desired solution accuracy

% Call dido
tStart= cputime;    % start CPU clock 
[cost, primal, dual] = dido(Brac_1, algorithm);
runTime = cputime-tStart
% Ta da!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          OUTPUT             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

V = primal.states(1,:);
v = primal.states(2,:);
theta = primal.states(3,:);
mfuel = primal.states(4,:);
t = primal.nodes;

%=============================================
figure;
plot(t, V, '-o', t, v, '-x', t, theta, '-.' ,t ,mfuel, '--');
title('Brachistochrone States: Brac 1')
xlabel('time');
ylabel('states');
legend('V', 'v', 'theta', 'mfuel');

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
%==============================================






% SHOLTO ADDITION: TESTING OPTIMALITY

%GRADIENT NORMALITY CONDITION


% Lagrangian of the Hamiltonian 
LH = dual.Hamiltonian + dot(dual.states,primal.states) + dual.controls.*primal.controls;

dLHdu = diff(LH)./diff(primal.controls);

