function dz = rocketDynamics(z,u,phase)


h = z(1,:);   %Height
v = z(2,:);   %Velocity
m = z(3,:);   %Mass
gamma = z(4,:);


if isnan(gamma)
    gamma = deg2rad(90)
end

T = u(1,:);        %Thrust

density = 1.474085291*(0.9998541833.^h);  %Data fit off of wolfram alpha


A1 = 0.0095;
A2 = 25;
A3 = 0.953;
A4 = 0.036;
speedOfSound = 280;  %(m/s)  %At 10 km altitude
mach = v/speedOfSound;
Cd = A1*atan(A2*(mach-A3))+A4;

%%%% Compute the drag:
Area = pi*3.66;  %(m^2) cross-sectional area (SpaceX F9 Falcon)
D = 0.5*Cd.*Area.*density.*v.^2;

%%%% Compute gravity from inverse-square law:
rEarth = 6.3674447e6;  %(m) radius of earth
mEarth = 5.9721986e24;  %(kg) mass of earth
G = 6.67e-11; %(Nm^2/kg^2) gravitational constant
g = G*mEarth./((h+rEarth).^2);

%%%% Complete the calculation:
global Tmax
% dm = -60*ones(1,length(h)).*T/Tmax.*Tmax/200000;   %mass rate
dm = -160*ones(1,length(h)).*T/Tmax;

alpha = 0*ones(1,length(h));

xi = 0*ones(1,length(h));
phi = 0*ones(1,length(h));
zeta = 0*ones(1,length(h));
L = 0*ones(1,length(h));


switch phase
    case 'prepitch'
    gamma = deg2rad(90)*ones(1,length(h)); % Control Trajectory Angle 
    case 'postpitch'
    %Do nothing
end


[dr,dxi,dphi,dgamma,dv,dzeta] = RotCoords(h+rEarth,xi,phi,gamma,v,zeta,L,D,T,m,alpha);

if isnan(dgamma)
dgamma = 0;
end

dz = [dr;dv;dm;dgamma];

end