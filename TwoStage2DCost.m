function [EndpointCost, RunningCost] = TwoStage2DCost(primal, algorithm)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cost function for Rocket-Scramjet-Rocket System

% This module contains the entire vehicle/system model, as well as defining
% the final cost

% The previous iteration of velocity is initially calculated using primals, and subsequently
% dynamic calculations are performed to produce next velocity step. These
% will eventually converge.

% Global variables are used to pass this velocity to the Dynamics file


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


global nodes
% =======================================================
% Vehicle Model:
% =======================================================
% =========================================================================================
global M
global q
global dfuel
global a
global Fd
global Fueldt
global Thrust
global lift
global flapdeflection
global Alpha
global scattered
global rho
global const
global scale
global grid
global Atmosphere
global iteration

iteration = iteration + 1;

V = primal.states(1, :)*scale.V ; % Scaled vertical position
v = primal.states(2,:)*scale.v ;
theta  = primal.states(3, :)*scale.theta; % Velocity angle
mfuel = primal.states(4,:)*scale.m ;
time = primal.nodes(1, :); % Time
% thetadot  = primal.controls(1, :)*scale.theta;
thetadot = primal.states(5,:)*scale.thetadot;
omegadot  = primal.controls(1,:)*scale.thetadot;

[dfuel, Fueldt, a, q, M, Fd, Thrust, flapdeflection, Alpha, rho,lift,Penalty] = VehicleModel(time, theta, V, v, mfuel, nodes,scattered,grid,const,thetadot, Atmosphere);

% THIRD STAGE ======================================================
% NEED TO WATCH THIS, IT CAN EXTRAPOLATE BUT IT DOESNT DO IT WELL

global ThirdStagePayloadMass
global alt_list
global gamma_list
global v_list
global payload_array

% if V(end) > 40000
% ThirdStagePayloadMass = gaussmf(V(end),[10000 40000])*interp3(alt_list,gamma_list,v_list,payload_array,40000, rad2deg(theta(end)), v(end),'cubic');
% elseif v(end) < 2000
% ThirdStagePayloadMass = gaussmf(v(end),[1000 2000])*interp3(alt_list,gamma_list,v_list,payload_array,V(end), rad2deg(theta(end)), 2000,'cubic');
%     
% else
% ThirdStagePayloadMass = interp3(alt_list,gamma_list,v_list,payload_array,V(end), rad2deg(theta(end)), v(end),'cubic');
% end

if V(end) > 40000
ThirdStagePayloadMass = gaussmf(V(end),[10000 40000])*interp3(alt_list,gamma_list,v_list,payload_array,40000, rad2deg(theta(end)), v(end));
elseif v(end) < 2000
ThirdStagePayloadMass = gaussmf(v(end),[1000 2000])*interp3(alt_list,gamma_list,v_list,payload_array,V(end), rad2deg(theta(end)), 2000);
    
else
ThirdStagePayloadMass = interp3(alt_list,gamma_list,v_list,payload_array,V(end), rad2deg(theta(end)), v(end));
end

% Define Cost =======================================================

% Endcost = -dfuel;

% tf = primal.nodes(end);     
% Endcost = tf;

% It is able to run with no cost at all:
if const == 3 
Endcost = 0;
end

if const == 1  || const == 12 || const == 13 || const == 14
Endcost =  - 0.01*mfuel(end) - ThirdStagePayloadMass;
end

EndpointCost = Endcost;

if const == 1  || const == 12 || const == 13 || const == 14
    
%smoothing functions (can be adjusted depending on needs, remove if not working
% omegadot = diff(thetadot)./diff(time);
%  RunningCost = [0 0.005*abs(omegadot)]; % for smoothing 50kPa
% RunningCost = [0 0.001*abs(omegadot)]; %for smoothing 45kPa and 55kPa and
% high drag

    RunningCost = Penalty + abs(omegadot);
%     RunningCost = Penalty ; % The Penalty function ensures that it does not go over 50kPa, but still allows it to search that space. 
    %Omegadot cost smooths the trajectory (sometimes), but also throws a SOL error and
    %reduces the validity of the optimisation result (so only use if you
    %are sure of the end trajectory shape)
% RunningCost = 0;
end

if const == 3 

omegadot = diff(thetadot)./diff(time);
     
% RunningCost =((q-50000).^2+1000000)/1000000 + [0 0.005*abs(omegadot)]; % if a cost does not work, try loosening it 
RunningCost =((q-50000).^2+2000000)/2000000 + [0 0.005*abs(omegadot)];


end


end
