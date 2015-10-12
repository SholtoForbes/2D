function Isp = RESTM12int(M1, T1, P1, mdotair, mdotfuel, a)
% Engine Interpolator for RESTM12 Data
% Reading from RESTM12DATA which has data arranged as so:
%M1 T1 P1 Me Pe

data = dlmread('RESTM12DATA.txt');      
      
MeSpline = scatteredInterpolant(data(:,1),data(:,2),data(:,3),data(:,4));
PeSpline = scatteredInterpolant(data(:,1),data(:,2),data(:,3),data(:,5));

% these splines to not extrapolate well. Check Pe and Me.


Me = MeSpline(M1, T1, P1);
Pe = PeSpline(M1, T1, P1);

A1 = 1; %inlet area
Ae = 1; %exit area

g = 9.81;

V1 = M1/a;
Ve = Me/a;


Isp = (mdotair*(Ve - V1) + (Pe*Ae - P1*A1))/(mdotfuel*g);


end


