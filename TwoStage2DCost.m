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
global Scalev
global ThetaScale
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

global communicator
global communicator_trim

%Gravity
g = 9.81;


VScaled = primal.states(1, :) ; % Scaled vertical position

HScaled = primal.states(2, :) ; % Scaled horizontal position

%velocity primal
vScaled = primal.states(3,:) ;

%Descaling
V = VScaled * ScaleV;
H = HScaled * ScaleH;
v = vScaled * Scalev;

% theta  = primal.controls(1, :)/ThetaScale; % Velocity angle
theta  = primal.states(4, :)/ThetaScale; % Velocity angle

time = primal.nodes(1, :); % Time



% [dfuel, v, m, q, M, v_array] = VehicleModel(time, theta, V, H, nodes);
%velocity primal
[dfuel, Fueldt, a, m, q, M, Fd, Thrust] = VehicleModel(time, theta, V, H, v, nodes, communicator, communicator_trim);

% THIRD STAGE ======================================================





% Define Cost =======================================================
global Endcost

% Endcost = -dfuel;

% tf = primal.nodes(end);     
% Endcost = tf;

% It is able to run with no cost at all:
Endcost = 0;

% Endcost = -gaussmf(theta(end),[0.01 0.1]) * 7.7e6;

EndpointCost = Endcost;

% RunningCost = 0;

% RunningCost =((q-50000).^2+1000000)/1000000; %this works
% RunningCost =((q-50000).^2+500000)/500000;

% RunningCost = -gaussmf(q,[1000 50000]); this doesnt work

RunningCost = Fueldt;
% 