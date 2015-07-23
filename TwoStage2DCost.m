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
[dfuel, Fueldt, a, m, q, M, Fd] = VehicleModel(time, theta, V, H, v, nodes, communicator, communicator_trim);

% THIRD STAGE ======================================================





% Define Cost =======================================================

% EndpointCost = -dfuel;

% tf = primal.nodes(end);     
% EndpointCost = tf;

% EndpointCost = abs(q-50000);

% It is able to run with no cost at all:
EndpointCost = 0;



% RunningCost = 0;

% RunningCost =((q-50000).^2+1000000)/1000000; %WORKS

RunningCost = Fueldt;
