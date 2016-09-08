clear all
% my attempt at third stage simulation... not very good

time1 = cputime;

Atmosphere = dlmread('atmosphere.txt');

%Simulating the Third Stage Rocket Trajectory

% Starting_Altitude = 33000; %m
% 
% Starting_Theta = .1; %rad

iteration = 1;

% for minit = 1500:100:3000


% for k = 20000:5000:60000
% for j = -.05:.01:0.1
% for u = 1800:200:3400

k = 32000;
j = 0.1;
u = 2800;

[k j u];
        
        Starting_Altitude = k;
        Starting_Theta = j;

        c = [];
        CD = [];
        CL = [];
        M = [];
        CN = [];
        CA = [];
        vx = [];
        vy = [];
        rho = [];
        t= [];
        Theta = [];
        Alt = [];
        mfuel = [];
        Hor = [];
        D = [];
        L = [];

HelioSync_Altitude = 566.89 + 6371; %Same as Dawids

r_E = 6371000; % earth radius

Orbital_Velocity_f = sqrt(398600/(566.89 + 6371))*10^3; %Calculating the necessary orbital velocity with altitude in km

% maxturnratepos = deg2rad(8)/80; %rad/s these are from dawids glasgow paper, need to figure out where these come from
% maxturnrateneg = -deg2rad(8)/110; %rad/s
% maxturnrateneg = -0.003;

%Reference area from Dawids Cadin
A = 0.87;

g = 9.81; %standard gravity

%define starting condtions
t(1) = 0.;

dt = .1; %time step

i=1;

%

r(1) = r_E + k;

Alt(1) = k;

xi(1) = 0;
    
phi(1) = -0.1314;

gamma(1) = deg2rad(j);

v(1) = u;

zeta(1) = deg2rad(84);


m(1) = 1750; %vehicle mass, less 1100kg fuel (to match Dawids glasgow paper)

mdot = 14.71; %fuel mass flow rate from dawid

mfuel(1) = 1100; %this is approx what dawid used (this is only fuel in atmosphere)
% mfuel(1) = 800

Alpha = deg2rad(10); % angle of attack, from dawids 6d

while gamma(i) > 0;
    
    if mfuel(i) > 0
        T = 50510;
        
        mfuel(i+1) = mfuel(i) - mdot*dt;
    else
        T = 0;

        mfuel(i+1) = mfuel(i);
    end
 
    
    if Alt(i) < 85000
        c(i) = spline( Atmosphere(:,1),  Atmosphere(:,5), Alt(i)); % Calculate speed of sound using atmospheric data

        rho(i) = spline( Atmosphere(:,1),  Atmosphere(:,4), Alt(i)); % Calculate density using atmospheric data
    else
        c(i) = spline( Atmosphere(:,1),  Atmosphere(:,5), 85000); % if altitude is over 85km, set values of atmospheric data not change. i will need to look at this

        rho(i) = 0;
    end
    
    q(i) = 1/2*rho(i)*v(i)^2;
    
    M(i) = v(i)/c(i);
    
    %calculate axial and normal coefficient, and then lift and drag
    %coefficients. from Dawid (3i)
    
    CA(i) = 0.346 + 0.183*M(i) - 0.058*M(i)^2 + 0.00382*M(i)^3;
    
    CN(i) = (5.006 - 0.519*M(i) + 0.031*M(i)^2)*Alpha;
    
    CD(i) = CA(i)*cos(Alpha) + CN(i)*sin(Alpha);
    
    CL(i) = CN(i)*cos(Alpha) - CA(i)*sin(Alpha);
    
    D(i) = 1/2*rho(i)*(v(i)^2)*A*CD(i);
    
    L(i) = 1/2*rho(i)*(v(i)^2)*A*CL(i);
   
    [rdot,xidot,phidot,gammadot,vdot,zetadot] = RotCoordsRocket(r(i),xi(i),phi(i),gamma(i),v(i),zeta(i),L(i),D(i),T,mfuel(i)+m(i),Alpha);
    
    
    if gammadot < 0
       
        Alpha = 0;
    end
    
    
    r(i+1) = r(i) + rdot*dt;
    
    Alt(i+1) = r(i+1) - r_E;
    
    xi(i+1) = xi(i) + xidot*dt;
    
    phi(i+1) = phi(i) + phidot*dt;
    
    gamma(i+1) = gamma(i) + gammadot*dt;
    
    v(i+1) = v(i) + vdot*dt;
    
    zeta(i+1) = zeta(i) + zetadot*dt;
    
    
    
    if q(i) < 10
        m(i+1) = 1750 - 302.8; %release of heat shield, from dawids glasgow paper
        exocond = true;
    else 
        m(i+1) = m(i);
        exocond = false;
    end
    
    t(i+1) = t(i) + dt;    
    i = i+1;
    
end

if exocond == false
    fprintf('Did not reach exoatmospheric conditions')
    m(end) = 1750 - 302.8;
end

%Hohmann Transfer, from Dawid (3i)
% mu = 3.987*10^14;
mu = 398600;
Rearth = 6371; %radius of earth

Omega_E = 7.2921e-5 ; % rotation rate of the Earth rad/s

vexo = sqrt((v(end)*sin(zeta(end)))^2 + (v(end)*cos(zeta(end)) + r(end)*Omega_E*cos(phi(end)))^2); %Change coordinate system when exoatmospheric, add velocity component from rotation of the Earth

inc = asin(v(end)*sin(pi-zeta(end))/vexo);  % orbit inclination angle

v12 = sqrt(mu / (Alt(end)/10^3 + Rearth))*10^3 - vexo;

v23 = sqrt(mu / (Alt(end)/10^3+ Rearth))*(sqrt(2*HelioSync_Altitude/((Alt(end)/10^3 + Rearth)+HelioSync_Altitude))-1)*10^3;

v34 = sqrt(mu / HelioSync_Altitude)*(1 - sqrt(2*(Alt(end)/10^3 + Rearth)/((Alt(end)/10^3 + Rearth)+HelioSync_Altitude)))*10^3;

dvtot = v12 + v23 + v34;

%as this is happening in a vacuum we can compute while delta v at once for
%fuel usage, tsiolkovsky rocket equation. this should have gravity maybe


g = 9.81;

Isp = 350; %s

m2 = m(end)/(exp(v12/(Isp*g)));

m3 = m2/(exp(v23/(Isp*g)));

m4 = m3/(exp(v34/(Isp*g)));

mpayload = m4 - 347.4; % subtract structural mass, from Dawids glasgow paper

% plot(Hor, Alt)

payload_matrix(iteration,1) = k ;
payload_matrix(iteration,2) = j ;
payload_matrix(iteration,3) = u ;
payload_matrix(iteration,4) = mpayload;

% payload_matrix(iteration,1) = minit ;
% payload_matrix(iteration,2) = k ;
% payload_matrix(iteration,3) = j ;
% payload_matrix(iteration,4) = u ;
% payload_matrix(iteration,5) = mpayload;

[k j u mpayload]


% thirdstage = fopen('thirdstage.dat','a+');
%         
% thirdstage_results = [num2str(Starting_Altitude,'%10.4e') ' ' num2str(Starting_Theta,'%10.4e') ' ' num2str(mpayload,'%10.4e') '\r\n'] ;
%         
% fprintf(thirdstage,thirdstage_results);
figure(1)
plot(t, Alt)
figure(2)
plot(t,v)

% iteration = iteration + 1;
%  end
% end
% end
% % end     
% dlmwrite('thirdstage.dat', payload_matrix,'delimiter','\t')


% dlmwrite('thirdstagewithmass.dat', payload_matrix,'delimiter','\t')

% time = cputime - time1
