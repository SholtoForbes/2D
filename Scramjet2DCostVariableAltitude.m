function [EndpointCost, RunningCost] = Brac1Cost(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint Cost for the Brac:1 Formulation of the Brachistochrone Prob
% Template for A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Scale

% =======================================================
% Vehicle Model:
% =======================================================
VScaled = primal.states(1,:) ; 
HScaled = primal.states(2,:) ; 


V = VScaled * Scale;
H = HScaled * Scale;


theta  = primal.controls(1,:); %acceleration in x plane

% PLACEHOLDER VELOCITY ==========================
% global v
% v = 1*theta./theta;
%========================================


% timeScaled = primal.nodes(1,:);
% time = timeScaled*Scale;
time = primal.nodes(1,:);

dt_array = time(2:end)-time(1:end-1); % Length of each timestep

dV_array = V(2:end)-V(1:end-1);

dH_array = H(2:end)-H(1:end-1);

v_array = sqrt(dV_array.^2 + dH_array.^2) ./ dt_array; % calculate velocity array using 'previous iteration'


%neglecting AoA, fixed reference frame
m = 5000.;
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


% For each alpha, spline force results for current dynamic pressure and
% Mach no

% AoA1 = [Out_force(1,:);Out_force(4,:);Out_force(7,:)];
% AoA1_spline = [spline(AoA1(:,3), AoA1(:,6), M)];

%WORK IN PROGRESS

% NEED TO INTRODUCE AoA FROM LIFT FORCE EQUALITY AND RESOLVE THRUST AND
% MOMENT


% THRUST AND MOTION ==================================================================
global M

% Calculating Mach No (Descaled)
M = v_array./c(1:end-1) ;

Fd = spline(M_array, Fd_array, M);
Flift = spline(M_array, Flift_array, M)  ;
My = spline(M_array, My_array, M)  ;

% Calculating Dynamic Pressure
global v
q = 0.5 * rho(1:end-1) .* (v_array .^2);

Thrust =  - Fd + g*sin(theta(1:end-1)) + 100.; % INCLUDES PLACEHOLDER TERM FOR CONSTANT ACCELERATION

a = ((Thrust - (- Fd + g*sin(theta(1:end-1)))) / m ) / Scale; % acceleration SCALED

% v(1) = 1000/Scale; % Initial Velocity SCALED THIS WILL NEED TO BE INTEGRATED WIH MULTI STAGE
v(1) = 1;
for i=2:length(a)+1
    
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

dfuel = sum(fuelchange_array); %total change in 'fuel' this is negative


% Define Cost =======================================================

EndpointCost = -dfuel;

% tf = primal.nodes(end);     
% EndpointCost = tf;


% It is able to run with no cost at all:
% EndpointCost = 0;





RunningCost = 0;

% That's it!
% Remember to fill the first output first!