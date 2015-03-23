%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Scramjet Flight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;		
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%===================
% Problem variables:
%-------------------
% states = x, y, vx, vy
% controls = theta, T
%===================

%---------------------------------------
% bounds the state and control variables
%---------------------------------------

xL = 0; xU = 30;
yL = 0; yU = 30;
vxL = 0; vxU = 30;
vyL = 0; vyU = 30;

bounds.lower.states = [xL; yL; vxL; vyL];
bounds.upper.states = [xU; yU; vxU; vyU];

bounds.lower.controls = [0; 0];
bounds.upper.controls = [pi/2; 20];

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

%Scaling, need to make these global variables!
X = 4
Y = 1

bounds.lower.events = [0; 0; 1; 1; 40/X; 10/Y; 1 ; 1];
% bounds.lower.events = [0; 0; 1; 1; 10/X; 10/Y; 1 ; 1];
bounds.upper.events = bounds.lower.events;      % equality event function bounds
% bounds.upper.events = [0; 0; 2; 2; 10/X; 10/Y; 3 ; 3];

%============================================
% Define the problem using DIDO expresssions:
%============================================
Brac_1.cost 		= 'test2DCost';
Brac_1.dynamics	    = 'test2DDynamics';
Brac_1.events		= 'test2DEvents';		
%Path file optional	

Brac_1.bounds       = bounds;
%====================================================

% Dont know how this changes the output yet...
algorithm.nodes		= [40];					    % represents some measure of desired solution accuracy

% Call dido
tStart= cputime;    % start CPU clock 
[cost, primal, dual] = dido(Brac_1, algorithm);
runTime = cputime-tStart
% Ta da!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          OUTPUT             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = primal.states(1,:);
y = primal.states(2,:);
vx = primal.states(3,:);
vy = primal.states(4,:);
t = primal.nodes;
theta = primal.controls(1,:);
tau = primal.controls(2,:);



figure(1)
subplot(3,3,1)
plot(x,y)
title('x-y')
subplot(3,3,2)
plot(t, vx)
title('vx')
subplot(3,3,3)
plot(t, vy)
title('vy')
subplot(3,3,4)
plot(t, tau)
title('tau')
subplot(3,3,5)
plot(t, theta)
title('theta')



lam1 = dual.dynamics(1,:);
lam2 = dual.dynamics(2,:);
lam3 = dual.dynamics(3,:);
lam4 = dual.dynamics(4,:);

subplot(3,3,6);
plot(t, [lam1; lam2; lam3; lam4]);
title('costates')
xlabel('time');
ylabel('costates');
legend('\lambda_1', '\lambda_2', '\lambda_3', '\lambda_4');

subplot(3,3,7)
H = dual.Hamiltonian(1,:);
plot(t,H);
title('Hamiltonian')