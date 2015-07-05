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
global m
global dfuel
global v_array

%Gravity
g = 9.81;


VScaled = primal.states(1,1:nodes(1)) ; % Scaled vertical position

HScaled = primal.states(2,1:nodes(1)) ; % Scaled horizontal position

%Descaling
V = VScaled * ScaleV;
H = HScaled * ScaleH;

theta  = primal.controls(1,1:nodes(1))/ThetaScale; % Velocity angle

time = primal.nodes(1,1:nodes(1)); % Time



[dfuel, v, m, q, M, v_array] = VehicleModel(time, theta, V, H, nodes);



% Define Cost =======================================================

EndpointCost = -dfuel;

% tf = primal.nodes(end);     
% EndpointCost = tf;


% It is able to run with no cost at all:
% EndpointCost = 0;

RunningCost = 0;
