function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% global Scale
global v

%changed notation to horizontal and vertical, x and y in plane of vehicle
VScaled = primal.states(1,:) ; 
HScaled = primal.states(2,:) ; 

theta  = primal.controls(1,:); %


% %=========================================================================================
% Calculate Derivative Terms
% v2(1:80)=1;
% dv = v-v2;

% v(1:80)=1;


VScaleddot = v.*sin(theta);
HScaleddot = v.*cos(theta);

% VScaleddot = sin(theta) + 0.*v; %THIS DOESNT WORK - WHY DO I NEED v?
% % VScaleddot = sin(theta);
% HScaleddot = cos(theta);



% %====================================================================


%======================================================
XDOT = [VScaleddot; HScaleddot];
%======================================================