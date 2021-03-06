function [EndpointCost, RunningCost] = Brac1Cost(primal, algorithm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cost function for Rocket-Scramjet-Rocket System

% This module contains the entire vehicle/system model, as well as defining
% the final cost

% The previous iteration of velocity is initially calculated using primals, and subsequently
% dynamic calculations are performed to produce next velocity step. These
% will eventually converge.

% Global variables are used to pass this velocity to the Dynamics file


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global ScaleH
global ScaleV
global ThetaScale
global MultiStage
global nodes
% =======================================================
% Vehicle Model:
% =======================================================
% =========================================================================================
global M
global v
global q
global StartingV
global m

%Gravity
g = 9.81;


VScaled = primal.states(1,1:nodes(1)) ; % Scaled vertical position

HScaled = primal.states(2,1:nodes(1)) ; % Scaled horizontal position

%Descaling
V = VScaled * ScaleV;
H = HScaled * ScaleH;

theta  = primal.controls(1,1:nodes(1))/ThetaScale; % Velocity angle

time = primal.nodes(1,1:nodes(1)); % Time

dt_array = time(2:end)-time(1:end-1); % Time change between each node pt

dV_array = V(2:end)-V(1:end-1); % Vertical position change between each node pt

dH_array = H(2:end)-H(1:end-1); % horizontal position change between each node pt

global v_array
v_array = sqrt(dV_array.^2 + dH_array.^2) ./ dt_array; % calculate velocity array using 'previous iteration'


%neglecting AoA, fixed reference frame

% m2 = 5000.;
% VARIABLE MASS TEST
if MultiStage == 1
% define mass flow rates
mdot1 = 0.;
mdot2 = 0.;

% Initial stage masses
m(1) = 5000;
m(nodes/2+1) = 1200;

% NEED TO CHANGE THIS TO SEPARATE AT A DISTINCT CONDITION
for i = 2:nodes/2
    m(i) = m(i-1) + mdot1*dt_array(i-1);
end

for i = nodes/2+1:nodes-1
    m(i) = m(i-1) + mdot2*dt_array(i-1);
end

    
% m(1:nodes/2) = 5000;
% m(nodes/2+1:nodes-1) = 1200;
else
% mdot = -100.;
mdot = 0.; 

m(1) = 5000; 
for i = 2:nodes-1
    m(i) = m(i-1) + mdot*dt_array(i-1);
end
    
% m(1:nodes-1) = 5000;
end


%======================================================
% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments

Iy = 1.; %%%% CHANGE THIS

% import data from force matrix
Out_force = dlmread('out_force.txt');

% import data from atmosphere matrix
Atmosphere = dlmread('atmosphere.txt');

% Calculate ablsolute height
StartingV = 15000; % THIS NEEDS TO BE CHANGED FOR VARIABLE HEIGHT

Vabs = V + StartingV;

% Calculate speed of sound using atmospheric data
c = spline( Atmosphere(:,1),  Atmosphere(:,5), Vabs);

% Calculate density using atmospheric data
rho = spline( Atmosphere(:,1),  Atmosphere(:,4), Vabs);

%an initial interpolator for the force values at a fixed Arot, alpha and
%dynamic pressure (0,  -0.0174532925199 (negative up) , 45000.0)

M_array = [4.5 , 5. , 5.5]; 
Fd_array = [-36427.6593981 , -42995.3909773 , -50209.1507264];
Flift_array = [ 26851.676829 , 25865.7310572 , 24420.6025981 ];
My_array = [305002.235162 , 256125.242712 , 196654.950117 ];


% Calculating Dynamic Pressure

q = 0.5 * rho(1:end-1) .* (v_array .^2);



% Calculating Mach No (Descaled)

M = v_array./c(1:end-1) ;

% For each alpha, spline force results for current dynamic pressure and
% Mach no

% This needs to be implemented with a more robust vehicle model, WILL ALL
% BE NaN for a vehicle model with not enough data
% for i=1:nodes-1
% theta_temp = theta(i); 
% M_temp = M(i);
% q_temp = q(i);
% m_temp = m(i);
%     
% AoA(i) = OutForce(theta_temp,M_temp,q_temp,m_temp);
% 
% end
% rho
% q
% M
% AoA

% THRUST AND MOTION ==================================================================


Fd = spline(M_array, Fd_array, M);
Flift = spline(M_array, Flift_array, M)  ;
My = spline(M_array, My_array, M)  ;




%VARIABLE THRUST WITH STAGES ----------------------------------------------
% if MultiStage == 1
% % Stage 2 -----------------------------------------------------------------
% Thrust(1:nodes/2) =  - Fd(1:nodes/2) + g*sin(theta(1:nodes/2))+ 200;
% % Stage 3 -----------------------------------------------------------------
% Thrust(nodes/2+1:nodes(1)-1) =  - Fd(nodes/2+1:nodes-1) + g*sin(theta(nodes/2+1:nodes-1)) + 100.;
% else
% Thrust(1:nodes-1) =  - Fd(1:nodes-1) + g*sin(theta(1:nodes-1))+ 200; % This thrust is created so that there is constant acceleration
% end

Thrust(1:nodes-1) =  80000;

% Acceleration ------------------------------------------------------------
a = ((Thrust - (- Fd + g*sin(theta(1:end-1)))) ./ m ); % acceleration 




% Velocity ----------------------------------------------------------------
% v(1) = 0.8;
v(1) = 2000;
for i=2:nodes(1)
    
    v(i) = a(i-1) * dt_array(i-1) + v(i-1);  % Velocity calculated stepwise
    
end

%===========================================================================
% Efficiency
% NOTE DYNAMIC PRESSURE DOES NOT CHANGE DRAG IN FORCE FILE INGO GAVE ME, so
% making the efficiency only reliant on a specific dynamic pressure is
% nonsensical

Efficiency = 1;
% Efficiency = 1 + Vabs(1:end-1)/100000000;
% Efficiency = (-(q(1:end)-50000.).^2 + 300000^2)/300000^2.;
%  Efficiency = (-(Vabs(1:end-1)-25000.).^2 + 300000^2)/300000^2.;

%Fuel rate of change
Fueldt = Thrust ./ Efficiency; % Temporary fuel rate of change solution, directly equated to thrust (should give correct efficiency result, but cannot analyse total fuel change accurately)

fuelchange_array = -Fueldt.*dt_array ; %Fuel change over each timestep

global dfuel
dfuel = sum(fuelchange_array); %total change in 'fuel' this is negative




% Define Cost =======================================================

EndpointCost = -dfuel;

% tf = primal.nodes(end);     
% EndpointCost = tf;


% It is able to run with no cost at all:
% EndpointCost = 0;

RunningCost = 0;
