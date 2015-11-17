%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Scramjet Flight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;		
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  no end const = 1 or Q end const = 2, mass end const = 3, q state variable
%  const == 4
global const
const = 1

% Inputs ============================================

communicator = importdata('communicatornew.txt');
% communicator = importdata('communicator.txt');

communicator_trim = importdata('communicator_trim.txt');
% communicator_trim = importdata('communicator_trim_extrapolate.txt');

%this produces simple splines for vehicle data
global AoA_spline
global flapdeflection_spline
global Dragq_spline
[AoA_spline, flapdeflection_spline, Dragq_spline] = LiftForceInterp(communicator,communicator_trim);


%engine splines for thrust and fuel usage
enginedata = dlmread('engineoutput_matrix');

global ThrustF_spline
ThrustF_spline= scatteredInterpolant(enginedata(:,1),enginedata(:,2),enginedata(:,3)); %interpolator for engine data (also able to extrapolate badly)
global FuelF_spline
FuelF_spline= scatteredInterpolant(enginedata(:,1),enginedata(:,2),enginedata(:,4)); %interpolator for engine data



ThirdStageData = dlmread('thirdstage.dat');
global ThirdStagePayloadSpline
% ThirdStagePayloadSpline = scatteredInterpolant(ThirdStageData(:,1),ThirdStageData(:,2),ThirdStageData(:,3));

% ThirdStagePayloadSpline = scatteredInterpolant(ThirdStageData(:,1),ThirdStageData(:,2),ThirdStageData(:,3),ThirdStageData(:,4));
ThirdStagePayloadSpline = scatteredInterpolant(ThirdStageData(:,1),ThirdStageData(:,2),ThirdStageData(:,3),ThirdStageData(:,5));

Atmosphere = dlmread('atmosphere.txt');
% rho derivative
global drho
j=1;
for i = 1:100:85000
drho(j,1) = i;
drho(j,2) = (spline(Atmosphere(:,1),  Atmosphere(:,4), i) - spline(Atmosphere(:,1),  Atmosphere(:,4), i-1))/1;
j = j+1;
end

%=============================================== 
%Second Stage
% V0 = 20000.; % 
V0 = 20000.;
% V0 = 0.;
Vf = 50000.; % Final values here are for guess and bounds, need to be fairly accurate
% Vf = 45000.;

H0 = 0.;
Hf = 700000.;

v0 = 1864.13; % 50kpa q at 27000m
% v0 = 2000;
% vf = 2979.83; % 50kpa q at 33000m
vf = 2900;
%dawids results have around 1 degree or under flight path angle
%===================
% Problem variables:
% control factor: omega
% variable trajectory
%===================
 
%========================================================

%---------------------------------------
% bound and scale the state and control variables
%---------------------------------------

% VL = -1.;
VL = V0;
VU = 1.0*Vf; 

% VL = 27000;
% VU = 40000; 

HL = -1.;
HU = 1.2*Hf;

vL = 1500;
% vL = 2000;
vU = 3100; % This limit must not cause the drag force to exceed the potential thrust of the vehicle by a large amount, otherwise DIDO will not solve

% thetaL = -.2; %  NEED TO WATCH THAT THIS IS NOT OVERCONSTRAINING
% thetaU = 1.6;

if const == 1
% thetaL = -0.1;
thetaL = -0.1;
% thetaL = 0.;
else
thetaL = -0.1; %  NEED TO WATCH THAT THIS IS NOT OVERCONSTRAINING
end
% thetaL = 0.;
% thetaU = 0.26; %15 degrees

% thetaU = 0.26/2;
if const == 1
% thetaU = 0.17;
% thetaU = 0.1;
thetaU = 0.05;
else
thetaU = 0.1;  
end
% thetaU = 0.4; 


% mfuelL = -3000;
mfuelL = 0;
mfuelU = 1000; % 

QL = 0;
% QU = 100*10^6; %joules, estimate
% QU = 50*10^6; %this should limit the max heat, this is arbitrary, based on previous results
% QU = 30*10^6;
QU = 10*10^6;

ql = 0;
qu = 100000;

if const == 1 
bounds.lower.states = [VL ; vL; thetaL; mfuelL-1];
% bounds.lower.states = [VL ; vL; thetaL; mfuelL-3000];
% bounds.lower.states = [VL ; vL; thetaL; mfuelL];
bounds.upper.states = [VU ; vU; thetaU; mfuelU+1];
% bounds.upper.states = [VU ; vU; thetaU; mfuelU];
end

if const == 2
bounds.lower.states = [VL ; vL; thetaL; mfuelL; -1];
bounds.upper.states = [VU ; vU; thetaU; mfuelU; QU*1.2];
end

if const == 3
bounds.lower.states = [VL ; vL; thetaL; mfuelL-3000];
% bounds.lower.states = [VL ; vL; thetaL; mfuelL-1];
% bounds.upper.states = [VU ; vU; thetaU; mfuelU+1];
bounds.upper.states = [VU ; vU; thetaU; mfuelU];
end

if const == 4
bounds.lower.states = [VL ; vL; thetaL; mfuelL-3000; ql];
bounds.upper.states = [VU ; vU; thetaU; mfuelU; qu];
end

% control bounds

% thetadotL = -0.05;
% thetadotU = 0.05;

thetadotL = -0.02;
thetadotU = 0.02;

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
if const == 1 
bounds.lower.events = [v0; mfuelU];
% bounds.lower.events = [v0; mfuelU; mfuelL];
end

if const == 2
bounds.lower.events = [v0; mfuelU; QL; QU];
end

if const == 3
% bounds.lower.events = [v0; vf; mfuelU]; 
bounds.lower.events = [v0; mfuelU; mfuelL];
end

if const == 4
bounds.lower.events = [v0; vf; mfuelU; 27000; 50000]; 
end

bounds.upper.events = bounds.lower.events;      % equality event function bounds



% bounds.lower.constraints = 30000;
% bounds.upper.constraints = 70000;
% PATH BOUNDS IF NECESSARY


%============================================
% Define the problem using DIDO expresssions:
%============================================
Brac_1.cost 		= 'TwoStage2DCost';
Brac_1.dynamics	    = 'TwoStage2DDynamics';
Brac_1.events		= 'TwoStage2DEvents';	
% Brac_1.Path         = 'TwoStage2DPath';
%Path file optional	

Brac_1.bounds       = bounds;


% Node Definition ====================================================

algorithm.nodes		= [89];	


global nodes

nodes = algorithm.nodes;


%  Guess =================================================================

tfGuess = tfMax; % this needs to be close to make sure solution stays withing Out_Force bounds

if const == 1
guess.states(1,:) = [25000 ,35000]; 
else
guess.states(1,:) = [0 ,Vf];
end

 %V
% guess.states(1,:) = [27000 ,27000]; %V doesnt work
% guess.states(1,:) = [25000 ,35000]; %V

guess.states(2,:) = [v0, vf]; %v

guess.states(3,:) = [atan((Vf-V0)/(Hf-H0)),atan((Vf-V0)/(Hf-H0))]; 

% guess.states(3,:) = [0.9*atan((Vf-V0)/(Hf-H0)),atan((Vf-V0)/(Hf-H0))]; 

% guess.states(4,:) = [mfuelU, mfuelU/2];
guess.states(4,:) = [mfuelU, 0];

if const == 2
guess.states(5,:) = [0, 50*10^6];
end

if const == 4
guess.states(5,:) = [50*10^3, 50*10^3];
end


guess.controls(1,:)    = [0,0]; 


guess.time        = [t0 ,tfGuess];



% Tell DIDO the guess.  Note: The guess-free option is not available when
% using "knots"
%========================
algorithm.guess = guess;
% algorithm.guess = primal_old;
% %========================
% algorithm.mode = 'accurate';
%=====================================================================================


% count

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

V = primal.states(1,:);

v = primal.states(2,:);

t = primal.nodes;

% theta = primal.controls(1,:);

theta = primal.states(3,:);
thetadot = primal.controls(1,:);
% 
mfuel = primal.states(4,:);

% Q = primal.states(5,:);

% theta = primal.controls(1,:);

% mfuel = primal.states(3,:);

%calculating for interest
% c = 300.; % this will need to be brought into line with vehicle model

% M = v./ScaleFactor/c;
global M
% global v_array
% v_array
global q
global Fd
global Fueldt

global Thrust
global flapdeflection
global Alpha
global ThirdStagePayloadMass

global heating_rate
global Q

dt = t(2:end)-t(1:end-1); % Time change between each node pt
FuelUsed = zeros(1,nodes-1);
FuelUsed(1) = dt(1)*Fueldt(1);
for i = 2:nodes-1
    FuelUsed(i) = dt(i).*Fueldt(i) + FuelUsed(i-1);
end


% figure out horizontal motion
H(1) = 0;
for i = 1:nodes-1
H(i+1) = v(i)*(t(i+1) - t(i))*cos(theta(i)) + H(i);
end

figure(1)

subplot(5,5,[1,10])
hold on
plot(H, V)
plot(H(algorithm.nodes(1)), V(algorithm.nodes(1)), '+', 'MarkerSize', 10, 'MarkerEdgeColor','r')
title('Trajectory (m)')

dim = [.7 .52 .2 .2];
annotation('textbox',dim,'string',{['Payload Mass: ', num2str(ThirdStagePayloadMass), ' kg'],['Second Stage Fuel Used: ' num2str(1000 - mfuel(end)) ' kg']},'FitBoxToText','on');  


subplot(5,5,11)
hold on
plot(t, v)
plot(t(algorithm.nodes(1)), v(algorithm.nodes(1)), '+', 'MarkerSize', 10, 'MarkerEdgeColor','r')
title('Velocity (m/s)')


subplot(5,5,12)
plot(t, M)
title('Mach no')

subplot(5,5,13)
plot(t, q)
title('Dynamic Pressure (pa)')

subplot(5,5,14)
hold on
plot(t, rad2deg(theta))
plot(t(algorithm.nodes(1)), rad2deg(theta(algorithm.nodes(1))), '+', 'MarkerSize', 10, 'MarkerEdgeColor','r')
title('Trajectory Angle (Deg)')



subplot(5,5,15)
plot(t, Fd)
title('Drag Force')

subplot(5,5,16)
hold on
plot(t, mfuel + 8755.1 - 994)
title('Vehicle Mass (kg)')



subplot(5,5,17)
plot(t, Thrust)
title('Thrust (N)')

Isp = Thrust./Fueldt./9.81;
IspNet = (Thrust-Fd)./Fueldt./9.81;

subplot(5,5,18)
plot(t, Isp)
title('Isp')

subplot(5,5,19)
plot(t, IspNet)
title('Net Isp')

subplot(5,5,20)
plot(t, flapdeflection)
title('Flap Deflection (deg)')

subplot(5,5,21)
plot(t, Alpha)
title('Angle of Attack (deg)')

subplot(5,5,22);
plot(t, dual.dynamics);
title('costates')
xlabel('time');
ylabel('costates');
legend('\lambda_1', '\lambda_2', '\lambda_3');

subplot(5,5,23)
Hamiltonian = dual.Hamiltonian(1,:);
plot(t,Hamiltonian);
title('Hamiltonian')

subplot(5,5,24)
hold on
plot(t, rad2deg(thetadot))
title('Trajectory Angle Change Rate (Deg/s)')

dim = [.8 .0 .2 .2];
annotation('textbox',dim,'string',{['Third Stage Thrust: ', num2str(50), ' kN'],['Third Stage Starting Mass: ' num2str(2850) ' kg'],['Third Stage Isp: ' num2str(350) ' s']},'FitBoxToText','on');  


figure(2)
subplot(2,6,[1,6])
hold on
plot(H, V,'Color','k')

title('Trajectory')
xlabel('Horizontal Position (m)')
ylabel('Vertical Position (m)')



for i = 1:floor(t(end)/30)
    [j,k] = min(abs(t-30*i));
    str = strcat(num2str(round(t(k))), 's');
    text(H(k),V(k),str,'VerticalAlignment','top', 'FontSize', 10);
    
    plot(H(k), V(k), '+', 'MarkerSize', 10, 'MarkerEdgeColor','k')
end

plot(H(end), V(end), 'o', 'MarkerSize', 10, 'MarkerEdgeColor','k')


text(H(end),V(end),'Third Stage Transition Point','VerticalAlignment','top', 'FontSize', 10);

dim = [.65 .45 .2 .2];
annotation('textbox',dim,'string',{['Payload Mass: ', num2str(ThirdStagePayloadMass), ' kg'],['Second Stage Fuel Used: ' num2str(1000 - mfuel(end)) ' kg']},'FitBoxToText','on');  

thirdstageexample_H = [0+H(end) (H(end)-H(end - 1))+H(end) 20*(H(end)-H(end - 1))+H(end) 40*(H(end)-H(end - 1))+H(end) 60*(H(end)-H(end - 1))+H(end) 80*(H(end)-H(end - 1))+H(end)]; %makes a small sample portion of an arbitrary third stage trajectory for example
thirdstageexample_V = [0+V(end) (V(end)-V(end - 1))+V(end) 20*((V(end)-V(end -1)))+V(end) 40*((V(end)-V(end -1)))+V(end) 60*((V(end)-V(end -1)))+V(end) 80*((V(end)-V(end -1)))+V(end)];
plot(thirdstageexample_H, thirdstageexample_V, 'LineStyle', '--','Color','k');

hold on
subplot(2,6,[7,9])
xlabel('time (s)')

hold on
ax1 = gca; % current axes


line(t, rad2deg(theta),'Parent',ax1,'Color','k', 'LineStyle','-')

line(t, M,'Parent',ax1,'Color','k', 'LineStyle','--')



line(t, v./(10^3),'Parent',ax1,'Color','k', 'LineStyle','-.')


line(t, q./(10^4),'Parent',ax1,'Color','k', 'LineStyle',':', 'lineWidth', 2.0)

% line(t, heating_rate./(10^5),'Parent',ax1,'Color','k', 'LineStyle',':', 'lineWidth', 2.0)
% 
% line(t, Q./(10^7),'Parent',ax1,'Color','k', 'LineStyle','-', 'lineWidth', 2.0)

% legend(ax1,  'Trajectory Angle (degrees)', 'Mach no', 'Velocity (m/s x 10^3)', 'Dynamic Pressure (Pa x 10^4)',  'Q (Mj x 10)')
legend(ax1,  'Trajectory Angle (degrees)', 'Mach no', 'Velocity (m/s x 10^3)', 'Dynamic Pressure (Pa x 10^4)')

subplot(2,6,[10,12])
xlabel('time (s)')
ax2 = gca;

line(t, Alpha,'Parent',ax2,'Color','k', 'LineStyle','-')

line(t, flapdeflection,'Parent',ax2,'Color','k', 'LineStyle','--')


line(t, mfuel./(10^2),'Parent',ax2,'Color','k', 'LineStyle','-.')


line(t, IspNet./(10^2),'Parent',ax2,'Color','k', 'LineStyle',':', 'lineWidth', 2.0)



legend(ax2, 'AoA (degrees)','Flap Deflection (degrees)', 'Fuel Mass (kg x 10^2)', 'Net Isp (s x 10^2)')


figure(3)


subplot(2,5,[1,5]);

line(t, dual.dynamics(1,:),'Color','k', 'LineStyle','-');
line(t, dual.dynamics(2,:),'Color','k', 'LineStyle','--');
line(t, dual.dynamics(3,:),'Color','k', 'LineStyle','-.');
line(t, dual.dynamics(4,:),'Color','k', 'LineStyle',':');
title('costates')
xlabel('time');
ylabel('Costates');
axis([0,t(end),-1,1])
legend('\lambda_1', '\lambda_2', '\lambda_3', '\lambda_4');

subplot(2,5,[6,10])
Hamiltonian = dual.Hamiltonian(1,:);
plot(t,Hamiltonian,'Color','k');
axis([0,t(end),-1,1])
title('Hamiltonian')

%visualise third stage trajectory
%calculate angle of attack by creating spline and interpolating
ThirdStageAoASpline = scatteredInterpolant(ThirdStageData(:,1),ThirdStageData(:,2),ThirdStageData(:,3),ThirdStageData(:,4));
AoA = ThirdStageAoASpline(V(end), rad2deg(theta(end)), v(end));
figure(4)
evalc('ThirdStageVisTrajectory(AoA, V(end), rad2deg(theta(end)), v(end));');


% hold on
% subplot(2,5,[6,10])
% plot(t, Alpha)



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




