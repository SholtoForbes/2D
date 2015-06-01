function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global H_input
global V_input
global theta_array
global StartingV
global theta_initial
global ScaleFactor

%changed notation to horizontal and vertical, x and y in plane of vehicle
v = primal.states(1,:) ; 

H = primal.states(2,:);


a  = primal.controls(1,:); %acceleration in x plane


% %Interpolating for V and descaling
% V = spline(H_input, V_input, H/ScaleFactor);
% 
% %Interpolating for theta
theta_temp = spline(H_input(2:end), theta_array, H(2:end));
theta = [theta_initial,theta_temp]; %Appending an initial term on
% 
%=========================================================================================
% Calculate Derivative Terms
vdot = a;

Hdot = v.*cos(theta);


%====================================================================


%======================================================
XDOT = [vdot; Hdot];
%======================================================