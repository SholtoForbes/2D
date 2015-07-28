function [D ] = OutForce(theta,M,q,m,S, communicator, communicator_trim)
% function [Alpha, D ,pitchingmoment] = OutForce(theta,M,q,m,S, communicator, communicator_trim)
% Out_force interpolator

% theta = 0.78
% 
% M = 5.3
% 
% q = 53000
% 
% Thrust = 50000
% 
% m = 5000
gravity = m*9.81;



M_array = communicator(:,1);

Alpha_array = communicator(:,2);
 
cL_array = communicator(:,3);


% L_array = q*S.*cL_array;

% F_sum =   L_array - gravity*cos(theta); % Array of force sums in velocity perpendicular direction

cL_search = gravity.*cos(theta)./(q*S); % for force sum to equate to zero, this is the cL required

% Alpha = griddata(M_array,F_sum, Alpha_array, M, 0);


cD_array = communicator(:,4);

% cD = griddata(M_array,Alpha_array, cD_array, M, Alpha);
% cD = griddata(M_array,F_sum, cD_array, M, 0);
cD = griddata(M_array,cL_array, cD_array, M, cL_search);

Body_Drag = q*S.*cD;

% pitchingmoment_array = communicator(:,5);

% pitchingmoment = griddata(M_array,Alpha_array, pitchingmoment_array, M, Alpha);

% 
% % 
%Calculate Flap Deflection Necessary

% Mtrim_array = communicator_trim(:,1);
% 
% Alphatrim_array = communicator_trim(:,2);
% 
% pitchingmomenttrim_array = communicator_trim(:,4);
% 
% Flapdeflection_array = communicator_trim(:,3);
% % 
% Flap_Drag = griddata(Mtrim_array, Alphatrim_array, pitchingmomenttrim_array, Flapdeflection_array, M, Alpha, -pitchingmoment);

% D = Body_Drag + Flap_Drag;

D = Body_Drag;




