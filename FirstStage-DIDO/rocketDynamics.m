function dz = rocketDynamics(z,u)


h = z(1,:);   %Height
v = z(2,:);   %Velocity
gamma = z(3,:);   %Mass
m = z(4,:);


if isnan(gamma)
    gamma = deg2rad(90);
end

T = u(1,:);        %Thrust

density = 1.474085291*(0.9998541833.^h);  %Data fit off of wolfram alpha

%%%% Drag coefficient, calculated from paper:
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

dm = -60*ones(1,length(h));   %mass rate


alpha = 0*ones(1,length(h));

xi = 0*ones(1,length(h));
phi = 0*ones(1,length(h));
zeta = 0*ones(1,length(h));
L = 0*ones(1,length(h));
gamma = deg2rad(90)*ones(1,length(h));


[dr,dxi,dphi,dgamma,dv,dzeta] = RotCoords(h+rEarth,xi,phi,gamma,v,zeta,L,D,T,m,alpha);

if isnan(dgamma)
dgamma = 0;
end

dz = [dr;dv;dgamma;dm];

end