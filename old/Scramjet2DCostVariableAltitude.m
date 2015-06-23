function [EndpointCost, RunningCost] = Brac1Cost(primal, algorithm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint Cost for the Brac:1 Formulation of the Brachistochrone Prob
% Template for A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Scale
global MultiStage
global nodes
% =======================================================
% Vehicle Model:
% =======================================================
% if MultiStage ==1
% VScaled1  = primal.states(1,1:nodes(1)-2);
% VScaled2  = primal.states(1,nodes(1):nodes(1)+nodes(2));
% VScaled  = [VScaled1,VScaled2];
% else
% VScaled = primal.states(1,:) ; 
% end

% SECOND STAGE
% =========================================================================================
VScaled2 = primal.states(1,1:nodes(1)) ; 

HScaled2 = primal.states(2,1:nodes(1)) ; 

V2 = VScaled2 * Scale;
H2 = HScaled2 * Scale;

theta2  = primal.controls(1,1:nodes(1)); %acceleration in x plane

time2 = primal.nodes(1,1:nodes(1));

dt_array2 = time2(2:end)-time2(1:end-1);

dV_array2 = V2(2:end)-V2(1:end-1);

dH_array2 = H2(2:end)-H2(1:end-1);


v_array2 = sqrt(dV_array2.^2 + dH_array2.^2) ./ dt_array2; % calculate velocity array using 'previous iteration'


%neglecting AoA, fixed reference frame

% m2 = 5000.;
% VARIABLE MASS TEST
m2(1:nodes(1)/2) = 5000;
m2(nodes(1)/2+1:nodes(1)-1) = 2000;


Cl2 = .1;
Cd2 = .1;
rho3 = 0.02;
A2 = 3.;
g = 9.81;

%======================================================
% Adding better scramjet dynamics, added 21/4/15
% communicator matrix is given in terms of forces and moments

Iy2 = 1.; %%%% CHANGE THIS

% import data from force matrix
Out_force2 = dlmread('out_force.txt');

% import data from atmosphere matrix
Atmosphere = dlmread('atmosphere.txt');

% Calculate ablsolute height
StartingV2 = 25000; % THIS NEEDS TO BE CHANGED FOR VARIABLE HEIGHT

Vabs2 = V2 + StartingV2;



% Calculate speed of sound using atmospheric data
c2 = spline( Atmosphere(:,1),  Atmosphere(:,5), Vabs2);

% Calculate density using atmospheric data
rho2 = spline( Atmosphere(:,1),  Atmosphere(:,4), Vabs2);

%an initial interpolator for the force values at a fixed Arot, alpha and
%dynamic pressure (0,  -0.0174532925199 (negative up) , 45000.0)

M_array = [4.5 , 5. , 5.5]; 
Fd_array = [-36427.6593981 , -42995.3909773 , -50209.1507264];
Flift_array = [ 26851.676829 , 25865.7310572 , 24420.6025981 ];
My_array = [305002.235162 , 256125.242712 , 196654.950117 ];


% For each alpha, spline force results for current dynamic pressure and
% Mach no

% AoA1 = [Out_force(1,:);Out_force(4,:);Out_force(7,:)];
% AoA1_spline = [spline(AoA1(:,3), AoA1(:,6), M)];

%WORK IN PROGRESS

% NEED TO INTRODUCE AoA FROM LIFT FORCE EQUALITY AND RESOLVE THRUST AND
% MOMENT


% THRUST AND MOTION ==================================================================
% global M

% Calculating Mach No (Descaled)

M2 = v_array2./c2(1:end-1) ;

Fd2 = spline(M_array, Fd_array, M2);
Flift2 = spline(M_array, Flift_array, M2)  ;
My2 = spline(M_array, My_array, M2)  ;

% Calculating Dynamic Pressure
global v
q2 = 0.5 * rho2(1:end-1) .* (v_array2 .^2);


% Thrust =  - Fd + g*sin(theta(1:end-1)) + 100.; % INCLUDES PLACEHOLDER TERM FOR CONSTANT ACCELERATION
% Thrust2 =  - Fd2 + g*sin(theta2(1:end-1)); % NO ACCELERATION


%VARIABLE THRUST TEST
Thrust2(1:nodes(1)/2) =  - Fd2(1:nodes(1)/2) + g*sin(theta2(1:nodes(1)/2))+ 200;
Thrust2(nodes(1)/2+1:nodes(1)-1) =  - Fd2(nodes(1)/2+1:nodes(1)-1) + g*sin(theta2(nodes(1)/2+1:nodes(1)-1)) + 100.;



a2 = ((Thrust2 - (- Fd2 + g*sin(theta2(1:end-1)))) ./ m2 ) / Scale; % acceleration SCALED

v(1) = 1;
for i=2:nodes(1)
    
    v(i) = a2(i-1) * dt_array2(i-1) + v(i-1);  % Velocity calculated stepwise
    
end

%===========================================================================

% Efficiency CHANGE THIS TO DYNAMIC PRESSURE RATHER THAN M
% Efficiency = (-(M(1:end-1)-5.).^2 +25.)/25.; % this is a simple inverse parabola centred around M=5 and going to zero at M=0 and M=10 and scaled so that it varies between 1 and 0
% Efficiency = (-(M(1:end-1)-8).^2 + 80.)/80.; % increasing the added value gives a smoother function
% Efficiency = 1;
Efficiency2 = 1 + V2(1:end-1)/100;

%Fuel rate of change
Fueldt2 = Thrust2 ./ Efficiency2; % Temporary fuel rate of change solution, directly equated to thrust (should give correct efficiency result, but cannot analyse total fuel change accurately)

fuelchange_array2 = -Fueldt2.*dt_array2 ; %Fuel change over each timestep


dfuel2 = sum(fuelchange_array2); %total change in 'fuel' this is negative



% THIRD STAGE
% ========================================================================================================================
% if MultiStage ==1
% 
%     VScaled3 = primal.states(1,nodes(1)+1:nodes(1)+nodes(2)) ; 
% 
%     HScaled3 = primal.states(2,nodes(1)+1:nodes(1)+nodes(2)) ; 
% 
%     V3 = VScaled3 * Scale;
%     H3 = HScaled3 * Scale;
% 
%     theta3  = primal.controls(1,nodes(1)+1:nodes(1)+nodes(2)); %acceleration in x plane
% 
%     time3 = primal.nodes(1,nodes(1)+1:nodes(1)+nodes(2));
% 
%     dt_array3 = time3(2:end)-time3(1:end-1);
% 
%     dV_array3 = V3(2:end)-V3(1:end-1);
% 
%     dH_array3 = H3(2:end)-H3(1:end-1);
% 
% 
%     v_array3 = sqrt(dV_array3.^2 + dH_array3.^2) ./ dt_array3; % calculate velocity array using 'previous iteration'
% 
% 
%     %neglecting AoA, fixed reference frame
%     m3 = 5000.;
%     Cl3 = .1;
%     Cd3 = .1;
%     rho3 = 0.02;
%     A3 = 3.;
%     g = 9.81;
% 
%     %======================================================
%     % communicator matrix is given in terms of forces and moments
% 
%     Iy3 = 1.; %%%% CHANGE THIS
% 
%     % import data from force matrix
%     Out_force3 = dlmread('out_force.txt');
% 
%     % Calculate ablsolute height
%     StartingV3 = 25000; % THIS NEEDS TO BE CHANGED FOR VARIABLE HEIGHT
% 
%     Vabs3 = V3 + StartingV3;
% 
%     % Calculate speed of sound using atmospheric data
%     c3 = spline( Atmosphere(:,1),  Atmosphere(:,5), Vabs3);
% 
%     % Calculate density using atmospheric data
%     rho3 = spline( Atmosphere(:,1),  Atmosphere(:,4), Vabs3);
% 
%     %an initial interpolator for the force values at a fixed Arot, alpha and
%     %dynamic pressure (0,  -0.0174532925199 (negative up) , 45000.0)
% 
%     M_array = [4.5 , 5. , 5.5]; 
%     Fd_array = [-36427.6593981 , -42995.3909773 , -50209.1507264];
%     Flift_array = [ 26851.676829 , 25865.7310572 , 24420.6025981 ];
%     My_array = [305002.235162 , 256125.242712 , 196654.950117 ];
% 
%     % THRUST AND MOTION ==================================================================
% %     global M
% 
%     % Calculating Mach No (Descaled)
% 
%     M3 = v_array3./c3(1:end-1) ;
% 
%     Fd3 = spline(M_array, Fd_array, M3);
%     Flift3 = spline(M_array, Flift_array, M3)  ;
%     My3 = spline(M_array, My_array, M3)  ;
% 
%     % Calculating Dynamic Pressure
%     
%     q3 = 0.5 * rho3(1:end-1) .* (v_array3 .^2);
% 
%     % Thrust =  - Fd + g*sin(theta(1:end-1)) + 100.; % INCLUDES PLACEHOLDER TERM FOR CONSTANT ACCELERATION
%     Thrust3 =  - Fd3 + g*sin(theta3(1:end-1));
% 
%     a3 = ((Thrust3 - (- Fd3 + g*sin(theta3(1:end-1)))) / m3 ) / Scale; % acceleration SCALED
% 
%     
%     v(nodes(1)+1) = v(nodes(1)); % repeat velocity over the multistage gap
%     for i=2:nodes(2)
% 
%         v(i + nodes(1)) = a3(i-1) * dt_array3(i-1) + v(i-1 + nodes(1));  % Velocity calculated stepwise
% 
%     end
% 
%     %===========================================================================
%     % Efficiency CHANGE THIS TO DYNAMIC PRESSURE RATHER THAN M
%     Efficiency3 = 1 + V3(1:end-1)/100;
% 
%     %Fuel rate of change
%     Fueldt3 = Thrust3 ./ Efficiency3; % Temporary fuel rate of change solution, directly equated to thrust (should give correct efficiency result, but cannot analyse total fuel change accurately)
% 
%     fuelchange_array3 = -Fueldt3.*dt_array3 ; %Fuel change over each timestep
% 
%     dfuel3 = sum(fuelchange_array3); %total change in 'fuel' this is negative
% 
% 
% end
% 
% 
% 
% 
% 


% Define Cost =======================================================
global dfuel
if MultiStage == 1
    dfuel = dfuel2 + dfuel3;
else
    dfuel = dfuel2;
end

EndpointCost = -dfuel;

% tf = primal.nodes(end);     
% EndpointCost = tf;


% It is able to run with no cost at all:
% EndpointCost = 0;





RunningCost = 0;

% That's it!
% Remember to fill the first output first!