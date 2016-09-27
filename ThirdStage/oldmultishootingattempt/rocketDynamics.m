function dz = rocketDynamics(z,u)
global SCALE
global Atmosphere

h = z(1,:);   %Height
v = z(2,:);   %Velocity
m = z(3,:);   %Mass
gamma = z(4,:);
alpha = z(5,:);

Isp = 350;

%Reference area from Dawids Cadin
A = 0.87;


dalphadt = u(1,:);

if isnan(gamma)
    gamma = 1.5708;
end


density = 1.474085291*(0.9998541833.^h);  %Data fit off of wolfram alpha

c = spline(Atmosphere(:,1),  Atmosphere(:,5), h);

M = v/c;

p = spline(Atmosphere(:,1),  Atmosphere(:,3), h);


CA = 0.346 + 0.183*M - 0.058*M^2 + 0.00382*M^3;
    
CN = (5.006 - 0.519*M + 0.031*M^2).*Alpha;

Cd = CA.*cos(Alpha) + CN.*sin(Alpha);

Cl = CN.*cos(Alpha) - CA.*sin(Alpha);

D = 0.5*Cd.*A.*density.*v.^2;
L = 0.5*Cl.*A.*density.*v.^2;

%%%% Compute gravity from inverse-square law:
rEarth = 6.3674447e6;  %(m) radius of earth
mEarth = 5.9721986e24;  %(kg) mass of earth
G = 6.67e-11; %(Nm^2/kg^2) gravitational constant
g = G*mEarth./((h+rEarth).^2);

%%%% Complete the calculation:

dm = -14.71*ones(1,length(h))*SCALE^2;

T = Isp*mdot*9.81*rEarth^2/(h+rEarth)^2 + (101325-p)*0.636;



xi = 0*ones(1,length(h));
phi = 0*ones(1,length(h));
zeta = 0*ones(1,length(h));




[dr,dxi,dphi,dgamma,dv,dzeta] = RotCoords(h+rEarth,xi,phi,gamma,v,zeta,L,D,T,m,alpha);

if isnan(dgamma)
dgamma = 0;
end

dz = [dr;dv;dm;dgamma;dalphadt];

end