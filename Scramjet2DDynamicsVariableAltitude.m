function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Scale
global v

%changed notation to horizontal and vertical, x and y in plane of vehicle
VScaled = primal.states(1,:) ; 
HScaled = primal.states(2,:) ; 

theta  = primal.controls(1,:); %


% %=========================================================================================
% Calculate Derivative Terms

% a = 1*theta./theta; % PLACEHOLDER a THIS IS SCALED % this is an array of a, with size same as theta

VScaleddot = v.*sin(theta);
HScaleddot = v.*cos(theta);


% %====================================================================


%======================================================
XDOT = [VScaleddot; HScaleddot];
%======================================================