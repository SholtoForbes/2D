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

global ThirdStagePayloadSpline

global heating_rate
global Q

global rho
global const

V = primal.states(1, :) ; % Scaled vertical position

%velocity primal
v = primal.states(2,:) ;

theta  = primal.states(3, :); % Velocity angle

mfuel = primal.states(4,:) ;
% mfuel = primal.states(3,:) ;


time = primal.nodes(1, :); % Time

% theta  = primal.controls(1, :);

[dfuel, Fueldt, a, q, M, Fd, Thrust, flapdeflection, Alpha, heating_rate, Q, rho] = VehicleModel(time, theta, V, v, mfuel, nodes,AoA_spline,flapdeflection_spline,Dragq_spline,ThrustF_spline,FuelF_spline, const);

% THIRD STAGE ======================================================
% NEED TO WATCH THIS, IT CAN EXTRAPOLATE BUT IT DOESNT DO IT WELL



global ThirdStagePayloadMass
% ThirdStagePayloadMass = ThirdStagePayloadSpline(V(end), theta(end));
ThirdStagePayloadMass = ThirdStagePayloadSpline(V(end), rad2deg(theta(end)), v(end));

% ThirdStageFuelCost = 50*(-theta(end)+15)/15 + 50*(-V(end)+37000)/37000;%arbitrary fuel cost for third stage


% Define Cost =======================================================



% Endcost = -dfuel;

% tf = primal.nodes(end);     
% Endcost = tf;

% It is able to run with no cost at all:
if const == 3 || const == 4
Endcost = 0;
end

if const == 1
% Endcost =  - mfuel(end) - ThirdStagePayloadMass;
Endcost =  - 0.01*mfuel(end) - ThirdStagePayloadMass;
% Endcost =  - 0.05*mfuel(end) - ThirdStagePayloadMass;
% Endcost =  - 0.1*mfuel(end) - ThirdStagePayloadMass;
% Endcost =  - ThirdStagePayloadMass;
% Endcost = 0 ;
end

% Endcost = -gaussmf(theta(end),[0.01 0.1]) * 7.7e6;

% Endcost = ThirdStageFuelCost;

EndpointCost = Endcost;

if const == 1
RunningCost = 0;

% RunningCost =((q-50000).^2+1000000)/1000000; 

% for i = 1:length(q)   
% if q(i) > 50000
% RunningCost(i) =((q(i)-50000).^2+100000000)/100000000;
% else
% RunningCost(i) = 0;
% end
% end

end

if const == 3 || const == 4
% RunningCost =((q-80000).^2+2000000)/2000000;
% RunningCost =((q-50000).^2+4000000)/4000000; % if a cost does not work, try loosening it 
% RunningCost =((q-50000).^2+2000000)/2000000; % if a cost does not work, try loosening it 
% RunningCost =((q-50000).^2+1500000)/1500000; 
RunningCost =((q-50000).^2+1000000)/1000000; 
% RunningCost =((q-50000).^2+500000)/500000;

end

% RunningCost = -gaussmf(q,[1000 50000]); this doesnt work

% RunningCost = Fueldt;
% 

% RunningCost = -mfuel;


% for i = 1:length(q)
%     
% if q(i) > 70000
%     
% % RunningCost(i) = 1;   
% RunningCost(i) =1*((q(i)-50000).^2+100000)/100000-1;
% 
% elseif q(i) < 30000
% % RunningCost(i) = 1;    
% RunningCost(i) =1*((q(i)-30000).^2+100000)/100000-1;
% else
%     
% RunningCost(i) = 0;
%     
% end
%     
% end