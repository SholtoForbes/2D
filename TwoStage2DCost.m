function [EndpointCost, RunningCost] = Brac1Cost(primal, algorithm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint Cost for the Brac:1 Formulation of the Brachistochrone Prob
% Template for A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global Scale
global MultiStage
global nodes
% =======================================================
% Vehicle Model:
% =======================================================
% =========================================================================================
VScaled = primal.states(1,1:nodes(1)) ; 

HScaled = primal.states(2,1:nodes(1)) ; 

V = VScaled * Scale;
H = HScaled * Scale;

theta  = primal.controls(1,1:nodes(1)); %acceleration in x plane

time = primal.nodes(1,1:nodes(1));

dt_array = time(2:end)-time(1:end-1);

dV_array = V(2:end)-V(1:end-1);

dH_array = H(2:end)-H(1:end-1);

global v_array
v_array = sqrt(dV_array.^2 + dH_array.^2) ./ dt_array; % calculate velocity array using 'previous iteration'


%neglecting AoA, fixed reference frame

% m2 = 5000.;
% VARIABLE MASS TEST
m(1:nodes(1)/2) = 5000;
m(nodes(1)/2+1:nodes(1)-1) = 2000;


Cl = .1;
Cd = .1;
rho = 0.02;
A = 3.;
g = 9.81;

%======================================================
% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments

Iy = 1.; %%%% CHANGE THIS

% import data from force matrix
Out_force = dlmread('out_force.txt');

% import data from atmosphere matrix
Atmosphere = dlmread('atmosphere.txt');

% Calculate ablsolute height
StartingV = 25000; % THIS NEEDS TO BE CHANGED FOR VARIABLE HEIGHT

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
global v
q = 0.5 * rho(1:end-1) .* (v_array .^2);

global M

% Calculating Mach No (Descaled)

M = v_array./c(1:end-1) ;

% For each alpha, spline force results for current dynamic pressure and
% Mach no

% This needs to be implemented with a more robust vehicle model 
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
% Stage 2 -----------------------------------------------------------------
Thrust(1:nodes(1)/2) =  - Fd(1:nodes(1)/2) + g*sin(theta(1:nodes(1)/2))+ 200;
% Stage 3 -----------------------------------------------------------------
Thrust(nodes(1)/2+1:nodes(1)-1) =  - Fd(nodes(1)/2+1:nodes(1)-1) + g*sin(theta(nodes(1)/2+1:nodes(1)-1)) + 100.;

% Acceleration ------------------------------------------------------------
a = ((Thrust - (- Fd + g*sin(theta(1:end-1)))) ./ m ) / Scale; % acceleration SCALED

% Velocity ----------------------------------------------------------------
v(1) = 0.8;
for i=2:nodes(1)
    
    v(i) = a(i-1) * dt_array(i-1) + v(i-1);  % Velocity calculated stepwise
    
end

%===========================================================================

% Efficiency CHANGE THIS TO DYNAMIC PRESSURE RATHER THAN M
% Efficiency = (-(M(1:end-1)-5.).^2 +25.)/25.; % this is a simple inverse parabola centred around M=5 and going to zero at M=0 and M=10 and scaled so that it varies between 1 and 0
% Efficiency = (-(M(1:end-1)-8).^2 + 80.)/80.; % increasing the added value gives a smoother function
% Efficiency = 1;
Efficiency = 1 + V(1:end-1)/100;

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
