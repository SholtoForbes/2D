function [Drag, Alpha, flapdeflection] = OutForce(theta,M,q,m,AoA_spline,flapdeflection_spline,Dragq_spline)
% function [Alpha, D ,pitchingmoment] = OutForce(theta,M,q,m,S, communicator, communicator_trim)
% Out_force interpolator

% find aerodynamics using only gravity of vehicle
gravity = -m*9.81; 

liftq_search = -gravity.*cos(theta)./q;

%use LiftForceInterp splines
Alpha = AoA_spline(M,liftq_search);
flapdeflection = flapdeflection_spline(M,liftq_search);
Drag = Dragq_spline(M,liftq_search).*q; 




