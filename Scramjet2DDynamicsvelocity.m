function XDOT = Brac1Dynamics(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global H_input
global V_input
global theta_array
global StartingV


%changed notation to horizontal and vertical, x and y in plane of vehicle
vH = primal.states(1,:) ;  
vV = primal.states(2,:);	

fuel = primal.states(3,:); % might need to be changed to a generic efficiency term

H = primal.states(4,:);


a  = primal.controls(1,:); %acceleration in x plane


%Interpolating for V
V = spline(H_input, V_input, H);

%Interpolating for theta
theta_temp = spline(H_input(2:end), theta_array, H(2:end));
theta = [0.78,theta_temp]; %Appending an initial term on, will need to change this to being defined by velocity



%=======================================================
% Vehicle Model:
%=======================================================

%neglecting AoA, fixed reference frame
m = 3000.;
Cl = .1;
Cd = .1;
rho = 0.02;
A = 3.;
g = 9.81;
%======================================================
% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments

Iy = 1.; %%%% CHANGE THIS

% import data from force matrix
Out_force = dlmread('out_force.txt');
% import data from atmosphere matrix
Atmosphere = dlmread('atmosphere.txt');


Vabs = V + StartingV;
c = spline( Atmosphere(:,1),  Atmosphere(:,5), Vabs);

% Mach no
% c = 300; % speed of sound (replace with atmosphere)
M = sqrt((vH).^2 + (vV).^2)./c ;



%an initial interpolator for the force values at a fixed Arot, alpha and
%dynamic pressure (0,  -0.0174532925199 (negative up) , 45000.0)

% 1

M_array = [4.5 , 5. , 5.5]; 
Fd_array = [-36427.6593981 , -42995.3909773 , -50209.1507264];
Flift_array = [ 26851.676829 , 25865.7310572 , 24420.6025981 ];
My_array = [305002.235162 , 256125.242712 , 196654.950117 ];



Fd = spline(M_array, Fd_array, M) ./ Vabs ; % drag force including fudge factor for altitude test!
% Fd = spline(M_array, Fd_array, M);
Flift = spline(M_array, Flift_array, M)  ;
My = spline(M_array, My_array, M)  ;



Thrust = a*m - Fd + g*sin(theta) ; % Thrust term



% Thrust =   Fx - Fd;



%=========================================================================================
% Calculate Derivative Terms
Hdot = vH;



vHdot = a.*cos(theta);   %will need to add initial angle term calculated from initial velocities
vVdot = a.*sin(theta); 



% fueldot = -0.001 .* abs(a); % this will need to be changed
fueldot = -0.0001 * Thrust;

%====================================================================


%======================================================
XDOT = [vHdot; vVdot;  fueldot; Hdot];
%======================================================