function [D, Alpha, flapdeflection] = OutForce(theta,M,q,m,A, Alpha_spline, Cd_spline, pitchingmoment_spline ,flapdeflection_spline,flapdrag_spline,flaplift_spline)
% function [Alpha, D ,pitchingmoment] = OutForce(theta,M,q,m,S, communicator, communicator_trim)
% Out_force interpolator

% find aerodynamics using only gravity of vehicle
gravity = -m*9.81; 

% cL_search_i = -gravity.*cos(theta)./(q*A); % for force sum to equate to zero, this is the cL required
% 
% body_pitchingmoment_i = pitchingmoment_spline(M, cL_search_i);
% 
% pitchingmoment_search_i = -body_pitchingmoment_i;
% 
% Alpha_i = Alpha_spline(M, cL_search_i);
% 
% Flap_lift_i = flaplift_spline(M,Alpha_i,pitchingmoment_search_i); %estimated flap lift
% 
% 
% %add estimated flap lift to gravity to get a closer approximation of actual
% %lift force needed. To improve this, more iterations of this may be added
% %if necessary, though it is expected that flap lift is around 5% of total
% %so this approximation should be good enough
% 
% gravity_modified = -m*9.81 + Flap_lift_i;

% cL_search = -gravity_modified.*cos(theta)./(q*A);
cL_search = -gravity.*cos(theta)./(q*A);


cD = Cd_spline(M, cL_search);

Body_Drag = q*A.*cD;

Alpha = Alpha_spline(M, cL_search);

% pitchingmoment = griddata(M_array,Alpha_array, pitchingmoment_array, M, Alpha);

body_pitchingmoment = pitchingmoment_spline(M, cL_search);


% 
% % 
%Calculate Flap Deflection Necessary

% Flap_Drag = griddata(Mtrim_array, Alphatrim_array, pitchingmomenttrim_array, Flapdeflection_array, M, Alpha, -pitchingmoment);
% SHOULD USE SCATTEREDINTERPOLANT FOR THIS

pitchingmoment_search = -body_pitchingmoment;

flapdeflection = flapdeflection_spline(M,Alpha,pitchingmoment_search);
Flap_Drag = flapdrag_spline(M,Alpha,pitchingmoment_search);
% Flap_lift = flaplift_spline(M,Alpha,pitchingmoment_search);


D = Body_Drag + Flap_Drag;

% D = Body_Drag;




