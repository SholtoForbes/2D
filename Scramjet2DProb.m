%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Scramjet Flight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;		
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Scaling

%================================================
TrueDh = 1000.; % True horizontal distance
TrueDv = 100.; % True vertical distance

ScaleDh = 10.; %scaled distance. 10 seems to be a consistently good value
ScaleDv = 10.;


global HScale
HScale = ScaleDh./TrueDh ; % Horizontal scale
global VScale
VScale = ScaleDv./TrueDv ; % Vertical scale
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

% velocity limits, scaled to problem. 
vhL = 0.*HScale; vhU = 10000.*HScale; 
vvL = -10000.*VScale; vvU = 10000.*VScale;



thetaL = -1.57; thetaU = 1.57; %these will need to be adjusted
omegaL = -1.0; omegaU = 1.;


%Added these as placeholders
% MU = 10.;
% ML = 0.;
% fuel, bounds are arbitrary
fuelL = 0;
fuelU = 1;


bounds.lower.states = [hL; vL; vhL; vvL; thetaL; omegaL; fuelL];
bounds.upper.states = [hU; vU; vhU; vvU; thetaU; omegaU; fuelU];

%ADJUSTED FOR NORMALISATION
bounds.lower.controls = [-200000.;-200000.; -100.];
bounds.upper.controls = [200000.;200000.; 100.]; % Control bounds, Unscaled

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


bounds.lower.events = [0; 0; ScaleDh; ScaleDv];
% bounds.lower.events = [0; 0; 0; 0;  ScaleDh; ScaleDv ; 0; 0]; %ADDED V BOUNDS< CANT GET THIS TO WORK


bounds.upper.events = bounds.lower.events;      % equality event function bounds
% bounds.upper.events = [0; 0; 7000*HScale; 7000*VScale;  ScaleDh; ScaleDv ; 7000*HScale; 7000*VScale];

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
algorithm.nodes		= [50];					    % represents some measure of desired solution accuracy

% algorith.mode = 'accurate';  %this did not seem to make a difference 28/4


% Guess
tfGuess = .15;  % this has been chosen to give an appropriate Mach no guess
%========================================================================
guess.states(1,:) = [0, ScaleDh/2, ScaleDh]; %H
guess.states(2,:) = [0, ScaleDv/2,  ScaleDv]; %V
guess.states(3,:) = [(ScaleDh-0)/(tfGuess-0), (ScaleDh-0)/(tfGuess-0), (ScaleDh-0)/(tfGuess-0) ]; %vh, Just basic derivatives fo now, constant
guess.states(4,:) = [(ScaleDh-0)/(tfGuess-0), (ScaleDv-0)/(tfGuess-0), (ScaleDh-0)/(tfGuess-0)]; %vv
guess.states(5,:) = [0.78,0.78,0.78]; %theta, guess set at 45 degrees
guess.states(6,:) = [0,0,0]; %omega
guess.states(7,:) = [1.,1.,1.]; %fuel
guess.controls(1,:)    = [0,0,0]; %Fx, these are net force so 0 guess
guess.controls(2,:)    = [0,0,0]; %Fz
guess.controls(3,:)    = [0,0,0]; %Mc
guess.time        = [t0, tfGuess/2, tfGuess];
%=======================================================================

% Tell DIDO the guess.  Note: The guess-free option is not available when
% using "knots"
%========================
algorithm.guess = guess;
%========================



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
% M = primal.states(7,:);

t = primal.nodes;

Fx = primal.controls(1,:);
Fz = primal.controls(2,:);
Mc = primal.controls(3,:);

%calculating for interest
c = 1000.;
M = sqrt((vh./HScale).^2 + (vv./VScale).^2)/c 


figure(1)
subplot(3,4,1)
plot(h,v)
title('h-v')
subplot(3,4,2)
plot(t, vh)
title('vh')
subplot(3,4,3)
plot(t, vv)
title('vv')

subplot(3,4,4)
plot(t, theta)
title('theta')
subplot(3,4,5)
plot(t, omega)
title('omega')
subplot(3,4,6)
plot(t, Mc)
title('Mc')
subplot(3,4,7)
plot(t, Fx)
title('Fx')
subplot(3,4,8)
plot(t, Fz)
title('Fz')
% subplot(3,4,8)
% plot(t, M)
% title('M')

lam1 = dual.dynamics(1,:);
lam2 = dual.dynamics(2,:);
lam3 = dual.dynamics(3,:);
lam4 = dual.dynamics(4,:);

subplot(3,4,9);
plot(t, [lam1; lam2; lam3; lam4]);
title('costates')
xlabel('time');
ylabel('costates');
legend('\lambda_1', '\lambda_2', '\lambda_3', '\lambda_4');

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











