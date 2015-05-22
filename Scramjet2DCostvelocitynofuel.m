function [EndpointCost, RunningCost] = Brac1Cost(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint Cost for the Brac:1 Formulation of the Brachistochrone Prob
% Template for A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global H_input
global V_input
global theta_array
global StartingV
global theta_initial
global ScaleFactor


%=======================================================
% Vehicle Model:
%=======================================================
v = primal.states(1,:) ; 

H = primal.states(2,:);

a  = primal.controls(1,:); %acceleration in x plane

% %Interpolating for V and descaling
V = spline(H_input, V_input, H/ScaleFactor);

% %Interpolating for theta
theta_temp = spline(H_input(2:end), theta_array, H(2:end));
theta = [theta_initial,theta_temp]; %Appending an initial term on

%neglecting AoA, fixed reference frame
m = 3000.;
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
Vabs = V + StartingV;

% Calculate speed of sound using atmospheric data
c = spline( Atmosphere(:,1),  Atmosphere(:,5), Vabs);

% Calculating mach No (Descaled)
M = v/ScaleFactor./c ;


%an initial interpolator for the force values at a fixed Arot, alpha and
%dynamic pressure (0,  -0.0174532925199 (negative up) , 45000.0)

M_array = [4.5 , 5. , 5.5]; 
Fd_array = [-36427.6593981 , -42995.3909773 , -50209.1507264];
Flift_array = [ 26851.676829 , 25865.7310572 , 24420.6025981 ];
My_array = [305002.235162 , 256125.242712 , 196654.950117 ];



% Fd = spline(M_array, Fd_array, M) ./ Vabs ; % drag force including fudge
% factor for altitude test! this fudge factor is too large... needs to be a
% fraction or something
Fd = spline(M_array, Fd_array, M);
Flift = spline(M_array, Flift_array, M)  ;
My = spline(M_array, My_array, M)  ;

% NEED TO INTRODUCE AoA FROM LIFT FORCE EQUALITY AND RESOLVE THRUST AND
% MOMENT


Thrust = a/ScaleFactor*m - Fd + g*sin(theta);  % Thrust term

% NEED TO INTRODUCE EFFICIENCY AND Isp

dt_array = primal.nodes(2:end)-primal.nodes(1:end-1);
fuelchange_array = -Thrust(1:end-1).*dt_array ; %fuel change set as directly proportional to thrust

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