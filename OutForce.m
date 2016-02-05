function [Drag, Alpha, flapdeflection] = OutForce(theta,M,q,m,AoA_spline,flapdeflection_spline,Dragq_spline,v,V)
% function [Alpha, D ,pitchingmoment] = OutForce(theta,M,q,m,S, communicator, communicator_trim)
% Out_force interpolator

v_H = v.*cos(theta);

% find aerodynamics using only gravity of vehicle
% gravity = -m*9.81; 
gravity = m.*(- 6.674e-11.*5.97e24./(V + 6371e3).^2 + v_H.^2./(V + 6371e3));

liftq_search = -gravity.*cos(theta)./q;

%use LiftForceInterp splines
Alpha = AoA_spline(M,liftq_search);
flapdeflection = flapdeflection_spline(M,liftq_search);
Drag = Dragq_spline(M,liftq_search).*q; 




