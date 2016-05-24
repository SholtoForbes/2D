function [Drag, Alpha, flapdeflection,lift_search] = OutForce(theta,M,q,m,AoA_spline,flapdeflection_spline,Drag_spline,Flap_pitchingmoment_spline,flap_interp,flapdrag_interp,v,V,thetadot,time)
% function [Alpha, D ,pitchingmoment] = OutForce(theta,M,q,m,S, communicator, communicator_trim)
% Out_force interpolator

v_H = v.*cos(theta);

% find aerodynamics using only gravity of vehicle
gravity = m.*(- 6.674e-11.*5.97e24./(V + 6371e3).^2 + v_H.^2./(V + 6371e3)); %Includes Gravity Variation and Centripetal Force 

lift_search = -gravity.*cos(theta);

%use LiftForceInterp splines
Alpha = AoA_spline(v,V,lift_search);
% flapdeflection = flapdeflection_spline(v,V,lift_search);
Body_Drag = Drag_spline(v,V,lift_search); 
Flap_pitchingmoment = Flap_pitchingmoment_spline(v,V,lift_search);

omegadot = diff(thetadot)./diff(time);
I = 150000; 
extramoment = [0 -omegadot*I];

flapdeflection = flap_interp(M,Alpha,Flap_pitchingmoment + extramoment);

flapdrag = flapdrag_interp(M,Alpha,Flap_pitchingmoment + extramoment);

Drag = Body_Drag + flapdrag;







