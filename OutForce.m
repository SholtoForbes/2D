function [D, Alpha] = OutForce(theta,M,q,m,S, Alpha_spline, Cd_spline, pitchingmoment_spline ,flapdeflection_spline,flapdrag_spline,flaplift_spline)
% function [Alpha, D ,pitchingmoment] = OutForce(theta,M,q,m,S, communicator, communicator_trim)
% Out_force interpolator

gravity = m*9.81;

cL_search = gravity.*cos(theta)./(q*S); % for force sum to equate to zero, this is the cL required



cD = Cd_spline(M, cL_search);

Body_Drag = q*S.*cD;


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
Flap_lift = flaplift_spline(M,Alpha,pitchingmoment_search);


D = Body_Drag + Flap_Drag;

% D = Body_Drag;




