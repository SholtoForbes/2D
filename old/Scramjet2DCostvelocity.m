function [EndpointCost, RunningCost] = Brac1Cost(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint Cost for the Brac:1 Formulation of the Brachistochrone Prob
% Template for A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = primal.nodes(end);     
EndpointCost = tf;

% dfuel = primal.states(2,end) - primal.states(2,1);     %change in fuel, this will be negative
% EndpointCost = -dfuel;

% It is able to run with no cost at all:
% EndpointCost = 0;

% RunningCost = primal.controls(1,:);
RunningCost = 0;

% That's it!
% Remember to fill the first output first!