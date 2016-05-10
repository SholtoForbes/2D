function [EndpointCost, RunningCost] = Brac1Cost(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint Cost for the Brac:1 Formulation of the Brachistochrone Prob
% Template for A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Vf = primal.states(1,end);   
% 
% EndpointCost = -vf;

vf = primal.states(2,end);   

% EndpointCost = -vf;

% t = primal.nodes(end);
% 
% EndpointCost = t;


% EndpointCost = (Vf - 47000)^2;

EndpointCost = 0;
RunningCost = 0;

% That's it!
% Remember to fill the first output first!