clear all

time1 = cputime;

Atmosphere = dlmread('atmosphere.txt');

%Simulating the Third Stage Rocket Trajectory

% Starting_Altitude = 33000; %m
% 
% Starting_Theta = .1; %rad

iteration = 1;

for k = 20000:1000:50000
    for j = -.5:.05:0.8
        
        Starting_Altitude = k;
        Starting_Theta = j;
        
        
   

        c = [];
        CD = [];
        CL = [];
        M = [];
        CN = [];
        CA = [];
        v = [];
        rho = [];
        t= [];
        theta = [];
        Alt = [];
        mfuel = [];
        Hor = [];


        

HelioSync_Altitude = 566.89 + 6371; %Same as Dawids

Orbital_Velocity_f = sqrt(398600/(566.89 + 6371))*10^3; %Calculating the necessary orbital velocity with altitude in km

maxturnratepos = deg2rad(7)/80; %rad/s these are from dawids glasgow paper, need to figure out where these come from
maxturnrateneg = -deg2rad(7)/110; %rad/s

%Reference area using diameter from Dawid (3i)
A = pi*(0.9/2)^2;


%define starting condtions
t(1) = 0.;

dt = .5; %time step

v(1) = 3000;

Theta(1) = Starting_Theta;

Alt(1) = Starting_Altitude;

Hor(1) = 0.;

i=1;

%
Thrust = 50000;
m = 1750; %vehicle mass, less 1100kg fuel (to match Dawids glasgow paper)


mdot = 14.71; %fuel mass flow rate from dawid

mfuel(1) = 1100; %this is approx what dawid used

c(1) = spline( Atmosphere(:,1),  Atmosphere(:,5), Alt(1)); % Calculate speed of sound using atmospheric data

rho(1) = spline( Atmosphere(:,1),  Atmosphere(:,4), Alt(1)); % Calculate density using atmospheric data

M(1) = v(1)/c(1);


Alpha = 0; %will need to change this to trim vehicle
    
CA(1) = 0.346 + 0.183*M(1) - 0.058*M(1)^2 + 0.00382*M(1)^3;

CN(1) = (5.006 - 0.519*M(1) + 0.031*M(1)^2)*Alpha;

CD(1) = CA*cos(Alpha) + CN*sin(Alpha);

CL(1) = CN*cos(Alpha) - CA*sin(Alpha);



while mfuel(i) > 0;
    t(i+1) = t(i) + dt;
    
    if t > 160
        m = 1750 - 302.8; %release of heat shield, from dawids glasgow paper
    end
    
    Alt(i+1) = Alt(i) + v(i)*sin(Theta(i))*dt;
    
    Hor(i+1) = Hor(i) + v(i)*cos(Theta(i))*dt; 
    
    Theta(i+1) = Theta(i) + maxturnratepos*dt;

    
    
    mfuel(i+1) = mfuel(i) - mdot*dt;
    
    if Alt(i) < 85000
    c(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,5), Alt(i)); % Calculate speed of sound using atmospheric data

    rho(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,4), Alt(i)); % Calculate density using atmospheric data
    else
    c(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,5), 85000); % if altitude is over 85km, set values of atmospheric data not change. i will need to look at this
    
    rho(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,4), 85000);
    
    end
    
    M(i+1) = v(i)/c(i);
    
    %calculate axial and normal coefficient, and then lift and drag
    %coefficients. from Dawid (3i)
    
    CA(i+1) = 0.346 + 0.183*M(i) - 0.058*M(i)^2 + 0.00382*M(i)^3;
    
    CN(i+1) = (5.006 - 0.519*M(i) + 0.031*M(i)^2)*Alpha;
    
    CD(i+1) = CA(i)*cos(Alpha) + CN(i)*sin(Alpha);
    
    CL(i+1) = CN(i)*cos(Alpha) - CA(i)*sin(Alpha);
    
    Fd = 1/2*rho(i)*v(i)^2*A;
    
    v(i+1) = v(i) + Thrust/(m+mfuel(i))*dt - Fd/(m+mfuel(i))*dt;
    
    i = i+1;
    
    
end

%after fuel burn out, turn to zero trajectory angle

while Theta(i) > 0; 

    t(i+1) = t(i) + dt;
    
    if t > 160
        m = 1750 - 302.8; %release of heat shield, from dawids glasgow paper
    end
    
    Alt(i+1) = Alt(i) + v(i)*sin(Theta(i))*dt;
    
    Hor(i+1) = Hor(i) + v(i)*cos(Theta(i))*dt; 
    
    Theta(i+1) = Theta(i) + maxturnrateneg*dt;

    
    if Alt(i) < 85000
    c(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,5), Alt(i)); % Calculate speed of sound using atmospheric data

    rho(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,4), Alt(i)); % Calculate density using atmospheric data
    else
    c(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,5), 85000); % if altitude is over 85km, set values of atmospheric data not change. i will need to look at this
    
    rho(i+1) = spline( Atmosphere(:,1),  Atmosphere(:,4), 85000);
    
    end
    
    M(i+1) = v(i)/c(i);
    
    %calculate axial and normal coefficient, and then lift and drag
    %coefficients. from Dawid (3i)
    
    CA(i+1) = 0.346 + 0.183*M(i) - 0.058*M(i)^2 + 0.00382*M(i)^3;
    
    CN(i+1) = (5.006 - 0.519*M(i) + 0.031*M(i)^2)*Alpha;
    
    CD(i+1) = CA(i)*cos(Alpha) + CN(i)*sin(Alpha);
    
    CL(i+1) = CN(i)*cos(Alpha) - CA(i)*sin(Alpha);
    
    Fd = 1/2*rho(i)*v(i)^2*A;
    
    v(i+1) = v(i) - Fd/(m)*dt;
    
    i = i+1;
    
end


%Hohmann Transfer, from Dawid (3i)
% mu = 3.987*10^14;
mu = 398600;
Rearth = 6371; %radius of earth

v12 = sqrt(mu / (Alt(end)/10^3 + Rearth))*10^3 - v(end);

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

payload_matrix(iteration,1) = Starting_Altitude ;
payload_matrix(iteration,2) = Starting_Theta ;
payload_matrix(iteration,3) = mpayload;


% thirdstage = fopen('thirdstage.dat','a+');
%         
% thirdstage_results = [num2str(Starting_Altitude,'%10.4e') ' ' num2str(Starting_Theta,'%10.4e') ' ' num2str(mpayload,'%10.4e') '\r\n'] ;
%         
% fprintf(thirdstage,thirdstage_results);



iteration = iteration + 1;
 end
end

        
dlmwrite('thirdstage.dat', payload_matrix,'delimiter','\t')


% time = cputime - time1
