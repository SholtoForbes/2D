function [EndpointCost, RunningCost] = Brac1Cost(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint Cost for the Brac:1 Formulation of the Brachistochrone Prob
% Template for A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% global v
% =======================================================
% Vehicle Model:
% =======================================================
V = primal.states(1,:) ; 

theta  = primal.controls(1,:); %acceleration in x plane

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
StartingV = 25000;
Vabs = V + StartingV;

% Calculate speed of sound using atmospheric data
c = spline( Atmosphere(:,1),  Atmosphere(:,5), Vabs);

% Calculating Mach No (Descaled)
global M
v = 3000; %CHANGE THIS, MAKE GLOBAL
M = v./c ;

% Calculate density using atmospheric data
rho = spline( Atmosphere(:,1),  Atmosphere(:,4), Vabs);

% Calculating Dynamic Pressure
q = 0.5 * rho .* (v .^2);


%an initial interpolator for the force values at a fixed Arot, alpha and
%dynamic pressure (0,  -0.0174532925199 (negative up) , 45000.0)

M_array = [4.5 , 5. , 5.5]; 
Fd_array = [-36427.6593981 , -42995.3909773 , -50209.1507264];
Flift_array = [ 26851.676829 , 25865.7310572 , 24420.6025981 ];
My_array = [305002.235162 , 256125.242712 , 196654.950117 ];


% For each alpha, spline force results for current dynamic pressure and
% Mach no

AoA1 = [Out_force(1,:);Out_force(4,:);Out_force(7,:)];
AoA1_spline = [spline(AoA1(:,3), AoA1(:,6), M)];

%WORK IN PROGRESS


% Fd = spline(M_array, Fd_array, M) ./ Vabs ; % drag force including fudge
% factor for altitude test! this fudge factor is too large... needs to be a
% fraction or something
Fd = spline(M_array, Fd_array, M);
Flift = spline(M_array, Flift_array, M)  ;
My = spline(M_array, My_array, M)  ;

% NEED TO INTRODUCE AoA FROM LIFT FORCE EQUALITY AND RESOLVE THRUST AND
% MOMENT


% Thrust = a*m - Fd + g*sin(theta);  % Thrust term
Thrust =  - Fd + g*sin(theta);  % Thrust term

% Thrust = Thrust./Thrust;  % makes it an array of 1s for testing (careful, it is sometimes NaN)

% Efficiency CHANGE THIS TO DYNAMIC PRESSURE RATHER THAN M
% Efficiency = (-(M(1:end-1)-5.).^2 +25.)/25.; % this is a simple inverse parabola centred around M=5 and going to zero at M=0 and M=10 and scaled so that it varies between 1 and 0
% Efficiency = (-(M(1:end-1)-8).^2 + 80.)/80.; % increasing the added value gives a smoother function
Efficiency = 1;

%Fuel rate of change
Fueldt = Thrust(1:end-1) ./ Efficiency; % Temporary fuel rate of change solution, directly equated to thrust (should give correct efficiency result, but cannot analyse total fuel change accurately)

dt_array = primal.nodes(2:end)-primal.nodes(1:end-1); % Length of each timestep
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