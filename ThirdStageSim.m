clear all

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
j = 0.17;
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
        Fd = [];
        Fl = [];


        

HelioSync_Altitude = 566.89 + 6371; %Same as Dawids

Orbital_Velocity_f = sqrt(398600/(566.89 + 6371))*10^3; %Calculating the necessary orbital velocity with altitude in km

maxturnratepos = deg2rad(8)/80; %rad/s these are from dawids glasgow paper, need to figure out where these come from
maxturnrateneg = -deg2rad(8)/110; %rad/s


%Reference area using diameter from Dawid (3i)
A = pi*(0.9/2)^2; % this is the same for lift and drag

g = 9.81; %standard gravity

%define starting condtions
t(1) = 0.;

dt = .5; %time step


Theta(1) = Starting_Theta;

vx(1) = u*cos(Theta(1));
vy(1) = u*sin(Theta(1));

Alt(1) = Starting_Altitude;

Hor(1) = 0.;

i=1;

%
Thrust = 50510;


m = 1750; %vehicle mass, less 1100kg fuel (to match Dawids glasgow paper)


mdot = 14.71; %fuel mass flow rate from dawid

mfuel(1) = 1100; %this is approx what dawid used

c(1) = spline( Atmosphere(:,1),  Atmosphere(:,5), Alt(1)); % Calculate speed of sound using atmospheric data

rho(1) = spline( Atmosphere(:,1),  Atmosphere(:,4), Alt(1)); % Calculate density using atmospheric data

M(1) = sqrt(vx(1)^2+vy(1)^2)/c(1);


Alpha = deg2rad(10); % angle of attack, from dawids 6d

    
CA(1) = 0.346 + 0.183*M(1) - 0.058*M(1)^2 + 0.00382*M(1)^3;

CN(1) = (5.006 - 0.519*M(1) + 0.031*M(1)^2)*Alpha;

CD(1) = CA*cos(Alpha) + CN*sin(Alpha);

CL(1) = CN*cos(Alpha) - CA*sin(Alpha);

Fd(1) = 1/2*rho(1)*(vx(1)^2+vy(1)^2)*A*CD(1);

Fl(1) = 1/2*rho(1)*(vx(1)^2+vy(1)^2)*A*CL(1);

while mfuel(i) > 0;
    t(i+1) = t(i) + dt;
    
    if Alt(i) > 70000
        m = 1750 - 302.8; %release of heat shield, from dawids glasgow paper
    end

%     if Alt(i) > 70000
% %         m = 1750 - 302.8; %release of heat shield, from dawids glasgow paper
%     m = minit - 302.8;
%     else
%         m = minit;
%     
%     end
%     
    Alt(i+1) = Alt(i) + vy(i)*dt;
    
    Hor(i+1) = Hor(i) + vx(i)*dt; 
    
    Theta(i+1) = Theta(i) + maxturnratepos*dt;

    
    
    mfuel(i+1) = mfuel(i) - mdot*dt;
    
    if Alt(i) < 85000
    c(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,5), Alt(i)); % Calculate speed of sound using atmospheric data

    rho(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,4), Alt(i)); % Calculate density using atmospheric data
    else
    c(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,5), 85000); % if altitude is over 85km, set values of atmospheric data not change. i will need to look at this
    
%     rho(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,4), 85000);
    rho(i+1) = 0;
    
    end
    
    M(i+1) = sqrt(vx(i)^2+vy(i)^2)/c(i);
    
    %calculate axial and normal coefficient, and then lift and drag
    %coefficients. from Dawid (3i)
    
    CA(i+1) = 0.346 + 0.183*M(i) - 0.058*M(i)^2 + 0.00382*M(i)^3;
    
    CN(i+1) = (5.006 - 0.519*M(i) + 0.031*M(i)^2)*Alpha;
    
    CD(i+1) = CA(i)*cos(Alpha) + CN(i)*sin(Alpha);
    
    CL(i+1) = CN(i)*cos(Alpha) - CA(i)*sin(Alpha);
    
    Fd(i+1) = 1/2*rho(i)*(vx(i)^2+vy(i)^2)*A*CD(i);
    Fl(i+1) = 1/2*rho(i)*(vx(i)^2+vy(i)^2)*A*CL(i);
    
%     v(i+1) = v(i) + Thrust/(m+mfuel(i))*dt - Fd(i)/(m+mfuel(i))*dt - g*sin(Theta(i))/(m+mfuel(i))*dt; % assumes that gravity is offset by body lift when horizontal

    vx(i+1) = sqrt(vx(i)^2+vy(i)^2)*cos(Theta(i)) - Fd(i)/(m+mfuel(i))*dt*cos(Theta(i)) - Fl(i)/(m+mfuel(i))*dt*sin(Theta(i)) + Thrust/(m+mfuel(i))*dt*cos(Theta(i)+Alpha);
    vy(i+1) = sqrt(vx(i)^2+vy(i)^2)*sin(Theta(i)) - Fd(i)/(m+mfuel(i))*dt*sin(Theta(i)) + Fl(i)/(m+mfuel(i))*dt*cos(Theta(i)) - g*dt + Thrust/(m+mfuel(i))*dt*sin(Theta(i)+Alpha);
    
    i = i+1;
    
    
end

%after fuel burn out, turn to zero trajectory angle

% while Theta(i) > 0; 
while vy(i) > 0;

    t(i+1) = t(i) + dt;
    
    if Alt(i) > 70000
        m = 1750 - 302.8; %release of heat shield, from dawids glasgow paper
    end

%     if Alt(i) > 70000
% %         m = 1750 - 302.8; %release of heat shield, from dawids glasgow paper
%     m = m - 302.8;
%     else
%         m = minit;
%     end
    
    Alt(i+1) = Alt(i) + vy(i)*dt;
    
    Hor(i+1) = Hor(i) + vx(i)*dt; 
    
    Theta(i+1) = Theta(i) + maxturnrateneg*dt;

    
    if Alt(i) < 85000
    c(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,5), Alt(i)); % Calculate speed of sound using atmospheric data

    rho(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,4), Alt(i)); % Calculate density using atmospheric data
    else
    c(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,5), 85000); % if altitude is over 85km, set values of atmospheric data not change. i will need to look at this
    
    rho(i+1) = 0;
    
    end
    
    M(i+1) = sqrt(vx(i)^2+vy(i)^2)/c(i);
    
    %calculate axial and normal coefficient, and then lift and drag
    %coefficients. from Dawid (3i)
    
    CA(i+1) = 0.346 + 0.183*M(i) - 0.058*M(i)^2 + 0.00382*M(i)^3;
    
    CN(i+1) = (5.006 - 0.519*M(i) + 0.031*M(i)^2)*Alpha;
    
    CD(i+1) = CA(i)*cos(Alpha) + CN(i)*sin(Alpha);
    
    CL(i+1) = CN(i)*cos(Alpha) - CA(i)*sin(Alpha);
    
    Fd(i+1) = 1/2*rho(i)*(vx(i)^2+vy(i)^2)*A*CD(i);
    
    Fl(i+1) = 1/2*rho(i)*(vx(i)^2+vy(i)^2)*A*CL(i);
    
    vx(i+1) = vx(i) - Fd(i)/(m)*dt*cos(Theta(i)) - Fl(i)/(m)*dt*sin(Theta(i));
    vy(i+1) = vy(i) - Fd(i)/(m)*dt*sin(Theta(i)) + Fl(i)/(m)*dt*cos(Theta(i)) - g*dt;
    i = i+1;
    
end


%Hohmann Transfer, from Dawid (3i)
% mu = 3.987*10^14;
mu = 398600;
Rearth = 6371; %radius of earth

% vend = sqrt(vx(end)^2+vy(end)^2);
vend = vx(end)

v12 = sqrt(mu / (Alt(end)/10^3 + Rearth))*10^3 - vend;

v23 = sqrt(mu / (Alt(end)/10^3+ Rearth))*(sqrt(2*HelioSync_Altitude/((Alt(end)/10^3 + Rearth)+HelioSync_Altitude))-1)*10^3;

v34 = sqrt(mu / HelioSync_Altitude)*(1 - sqrt(2*(Alt(end)/10^3 + Rearth)/((Alt(end)/10^3 + Rearth)+HelioSync_Altitude)))*10^3;

dvtot = v12 + v23 + v34;

%as this is happening in a vacuum we can compute while delta v at once for
%fuel usage, tsiolkovsky rocket equation. this should have gravity maybe


g = 9.81;

Isp = 350; %s

m2 = m/(exp(v12/(Isp*g)));

m3 = m2/(exp(v23/(Isp*g)));

m4 = m3/(exp(v34/(Isp*g)));

mpayload = m4 - 347.4; % subtract structural mass from Dawids glasgow paper

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
plot(Hor, Alt)
figure(2)
plot(t,sqrt(vx.^2 + vy.^2))

% iteration = iteration + 1;
%  end
% end
% end
% % end     
% dlmwrite('thirdstage.dat', payload_matrix,'delimiter','\t')


% dlmwrite('thirdstagewithmass.dat', payload_matrix,'delimiter','\t')

% time = cputime - time1
