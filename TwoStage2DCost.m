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


global Stage
global nodes
% =======================================================
% Vehicle Model:
% =======================================================
% =========================================================================================
global M
% global v
global q
global m
global dfuel
% global v_array
global a
global Fd
global Fueldt
global Thrust

global flapdeflection
global Alpha


global AoA_spline
global flapdeflection_spline
global Dragq_spline


global ThrustF_spline
global FuelF_spline


V = primal.states(1, :) ; % Scaled vertical position

%velocity primal
v = primal.states(2,:) ;

% theta  = primal.states(3, :); % Velocity angle

% mfuel = primal.states(4,:) ;
mfuel = primal.states(3,:) ;


time = primal.nodes(1, :); % Time

theta  = primal.controls(1, :);

[dfuel, Fueldt, a, q, M, Fd, Thrust, flapdeflection, Alpha] = VehicleModel(time, theta, V, v, mfuel, nodes,AoA_spline,flapdeflection_spline,Dragq_spline,ThrustF_spline,FuelF_spline);

% THIRD STAGE ======================================================
% NEED TO WATCH THIS, IT CAN EXTRAPOLATE BUT IT DOESNT DO IT WELL

% ThirdStageData = dlmread('thirdstage.dat');
% 
% ThirdStageFuelSpline = scatteredInterpolant(ThirdStageData(:,1),ThirdStageData(:,2),ThirdStageData(:,3));
% 
% ThirdStageFuelCost = ThirdStageFuelSpline(theta(end), V(end));



% Define Cost =======================================================
global Endcost

% Endcost = -dfuel;

% tf = primal.nodes(end);     
% Endcost = tf;

% It is able to run with no cost at all:
Endcost = 0;

% Endcost = 2000 - mfuel(end); % change 2000 to whatever mU is

% Endcost = -gaussmf(theta(end),[0.01 0.1]) * 7.7e6;

% Endcost = ThirdStageFuelCost;

EndpointCost = Endcost;

% RunningCost = 0;

% RunningCost =((q-80000).^2+2000000)/2000000;
% RunningCost =((q-50000).^2+4000000)/4000000; % if a cost does not work, try loosening it 
RunningCost =((q-50000).^2+2000000)/2000000; % if a cost does not work, try loosening it 
% RunningCost =((q-50000).^2+1000000)/1000000; 
% RunningCost =((q-50000).^2+500000)/500000;

% RunningCost = -gaussmf(q,[1000 50000]); this doesnt work

% RunningCost = Fueldt;
% 

% RunningCost = -mfuel;