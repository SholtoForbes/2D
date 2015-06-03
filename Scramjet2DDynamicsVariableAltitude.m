function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Scale

%changed notation to horizontal and vertical, x and y in plane of vehicle
VScaled = primal.states(1,:) ; 
HScaled = primal.states(2,:) ; 
vScaled = primal.states(3,:);

theta  = primal.controls(1,:); %


% %=========================================================================================
% Calculate Derivative Terms
a = 1; % PLACEHOLDER a THIS IS SCALED
vScaleddot = a *theta./theta; % this is an array of a, with size same as theta


VScaleddot = vScaled.*sin(theta);
HScaleddot = vScaled.*cos(theta);


% %====================================================================


%======================================================
XDOT = [VScaleddot; HScaleddot; vScaleddot];
%======================================================