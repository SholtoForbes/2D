%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;				% always a good idea to begin with this!

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%
mTotal = 15000;   %(kg)  %Total lift-off mass
mFuel = 0.8*mTotal;  %(kg)  %mass of the fuel
mEmpty = mTotal-mFuel;  %(kg)  %mass of the rocket (without fuel)
Tmax = 200000;    %(N)   %Maximum thrust

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Pre-Pitchover Simulation                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
h0_prepitch = 0;  %Rocket starts on the ground
v0_prepitch = 0;  %Rocket starts stationary
m0_prepitch = mTotal;  %Rocket starts full of fuel
gamma0_prepitch = deg2rad(90);

tspan = [0 20];
prepitch0 = [h0_prepitch, v0_prepitch, gamma0_prepitch, m0_prepitch];
[t_prepitch, prepitch] = ode45(@(t,prepitch) rocketDynamics(prepitch,Tmax), tspan, prepitch0);


% tspan = [0 100];
% postpitch0 = [prepitch(end,1), prepitch(end,2), 89, prepitch(end,4)];
% [t_prepitch, postpitch] = ode45(@(t,postpitch) rocketDynamics(postpitch,Tmax), tspan, postpitch0);

prepitch
% postpitch

% 
% %---------------------------------------
% % bounds the state and control variables
% %---------------------------------------
% 
% 
% hL = 0; hU = 100000;
% vL = 0; vU = 3000;
% gammaL = 0; gammaU = deg2rad(90);
% mfuelL = 0; mfuelU = mFuel;
% 
% bounds.lower.states = [hL; vL; gammaL; mfuelL];
% bounds.upper.states = [hU; vU; gammaU; mfuelU];
% 
% bounds.lower.controls = [0];
% bounds.upper.controls = [Tmax];
% 
% 
% %------------------
% % bound the horizon
% %------------------
% t0	    = 0;
% tfMax 	= 1000;   % swag for max tf; DO NOT set to Inf even for time-free problems
% 
% bounds.lower.time 	= [t0; t0];				
% bounds.upper.time	= [t0; tfMax];			    % Fixed time at t0 and a possibly free time at tf
% 
% 
% %-------------------------------------------
% % Set up the bounds on the endpoint function
% %-------------------------------------------
% % See events file for definition of events function
% 
% % bounds.lower.events = [y(end,1); y(end,2); 89; y(end,3)-mStruct; 20000; 0];
% bounds.lower.events = [y(end,1); y(end,2); 89; y(end,3)-mStruct];
% bounds.upper.events = bounds.lower.events;      % equality event function bounds
% 
% 
% %============================================
% % Define the problem using DIDO expresssions:
% %============================================
% Brac_1.cost 		= 'Brac1Cost';
% Brac_1.dynamics	    = 'Brac1Dynamics';
% Brac_1.events		= 'Brac1Events';		
% %Path file not required for this problem formulation;	
% 
% Brac_1.bounds       = bounds;
% %====================================================
% 
% algorithm.nodes		= [50];					    % represents some measure of desired solution accuracy
% 
% % Call dido
% tStart= cputime;    % start CPU clock 
% [cost, primal, dual] = dido(Brac_1, algorithm);
% runTime = cputime-tStart
% % Ta da!
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %          OUTPUT             %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% h = primal.states(1,:);
% v = primal.states(2,:);
% gamma = primal.states(3,:);
% mfuel = primal.states(4,:);
% T  = primal.controls;
% t = primal.nodes;
% 
% %=============================================
% figure;
% plot(t, h, '-o', t, v, '-x', t, gamma, '-.' ,t ,mfuel, '--',t,T);
% title('Brachistochrone States: Brac 1')
% xlabel('time');
% ylabel('states');
% legend('V', 'v', 'theta', 'mfuel', 'Thrust');
% 
% %----------------------------------------------
% figure;
% plot(t, dual.Hamiltonian);
% title('Brachistochrone Hamiltonian Evolution');
% legend('H');
% xlabel('time');
% ylabel('Hamiltonian Value');
% 
% %----------------------------------------------
% figure;
% plot(t, dual.dynamics);
% title('Brachistochrone Costates: Brac 1')
% xlabel('time');
% ylabel('costates');
% legend('\lambda_x', '\lambda_y', '\lambda_v');
% %==============================================
% 
% 
% 
% 
% 
% 
% % SHOLTO ADDITION: TESTING OPTIMALITY
% 
% %GRADIENT NORMALITY CONDITION
% 
% 
% % Lagrangian of the Hamiltonian 
% LH = dual.Hamiltonian + dot(dual.states,primal.states) + dual.controls.*primal.controls;
% 
% dLHdu = diff(LH)./diff(primal.controls);

