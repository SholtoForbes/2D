function [rdot,xidot,phidot,gammadot,vdot,zetadot] = RotCoords(r,xi,phi,gamma,v,zeta,m,gammadot,grid, const, rho)
% Determination of motion in rotating coordinates

%xi  Longitude (rad)
%phi  Latitude (rad)
%gamma  Flight Path Angle (rad)
%zeta  Heading Angle (rad)

mu_E = 3.986e14; % m^3/s^2 Earth Gravitational Parameter
omega_E = 7.292115e-5; % s^-1 Earth Rotation Rate


lift_search = gammadot*m*v - (v/r - mu_E/(r^2*v))*cos(gamma)*m*v - m*v*cos(phi)*(2*omega_E*cos(zeta) + omega_E^2*r/v*(cos(phi)*cos(gamma)+sin(phi)*sin(gamma)*sin(zeta)));

alpha = scattered.AoA(v,V,lift_search);
D = scattered.drag(v,V,lift_search); 


if const == 1
    if q < 50000
%     if q(i) < 55000
%     if q(i) < 45000
    Efficiency = rho/(50000*2/v^2); % dont change this
    else
%         Efficiency(i) = .9; % for 45kPa
    Efficiency = 1; % for 50kPa
%     Efficiency(i) = 1.1; % for 55kPa
% %     Efficiency(i) = 1.2; 
    end
else       
    Efficiency = rho./(50000*2./v.^2); % linear rho efficiency, scaled to rho at 50000kpa
end

T = interp2(grid.Mgrid_eng2,grid.alpha_eng2,grid.T_eng,M,alpha,'spline').*cos(deg2rad(alpha)).*Efficiency;



rdot = v*sin(gamma);

xidot = v*cos(gamma)*cos(zeta)/(r*cos(phi));

phidot = v*cos(gamma)*sin(zeta)/r;

% gammadot = T*sin(alpha)/(m*v) + (v/r - mu_E/(r^2*v))*cos(gamma) + L/(m*v) + cos(phi)*(2*omega_E*cos(zeta) + omega_E^2*r/v*(cos(phi)*cos(gamma)+sin(phi)*sin(gamma)*sin(zeta)));

vdot = T*cos(alpha)/(m) - mu_E*sin(gamma)/r^2 -D/m + omega_E^2*r*cos(phi)*(cos(phi)*cos(gamma)+sin(phi)*sin(gamma)*sin(zeta)); 

zetadot = -v/r*tan(phi)*cos(gamma)*cos(zeta) + 2*omega_E*cos(phi)*tan(gamma)*sin(zeta) - omega_E^2*r/(v*cos(gamma))*sin(phi)*cos(phi)*cos(zeta)-2*omega_E*sin(phi);

end

