clear all

%Simulating the Third Stage Rocket Trajectory

Starting_Altitude = 33000; %m

Starting_Theta = .1; %rad

HelioSync_Altitude = 566.89*10^3; %Same as Dawids

Orbital_Velocity = sqrt(398600/566.89)*10^3; %Calculating the necessary orbital velocity with altitude in km

maxturnrate = 0.1; %rad/s

%define starting condtions
t(1) = 0.;

dt = .001; %time step

v(1) = 3000;

Theta(1) = Starting_Theta;

Alt(1) = Starting_Altitude;

Hor(1) = 0.;

i=1;

%
Thrust = 50000;
m = 100; %engine mass


mdot = 14.71; %fuel mass flow rate

mfuel(1) = 800; %this is approx what dawid used


while Theta < deg2rad(90);
    t(i+1) = t(i) + dt;
    
    Alt(i+1) = Alt(i) + v(i)*sin(Theta(i))*dt;
    
    Hor(i+1) = Hor(i) + v(i)*cos(Theta(i))*dt; 
    
    Theta(i+1) = Theta(i) + maxturnrate*dt;

    v(i+1) = v(i) + Thrust/(m+mfuel(i))*dt;
    
    mfuel(i+1) = mfuel(i) - 14.71*dt;
    
    i = i+1;
    
    
end

while Alt < 500*10^3; % this puts the third stage at about the right altitude for a turn rate of .1 rad/s
    t(i+1) = t(i) + dt;
    
    Alt(i+1) = Alt(i) + v(i)*sin(Theta(i))*dt;
    
    Hor(i+1) = Hor(i) + v(i)*cos(Theta(i))*dt; 
    
    Theta(i+1) = Theta(i);

    v(i+1) = v(i) + Thrust/(m+mfuel(i))*dt;
    
    mfuel(i+1) = mfuel(i) - 14.71*dt;
    
    i = i+1;
end


while v < Orbital_Velocity
    t(i+1) = t(i) + dt;
    
    
    Alt(i+1) = Alt(i) + v(i)*sin(Theta(i))*dt;
    
    Hor(i+1) = Hor(i) + v(i)*cos(Theta(i))*dt; 
    
    Theta(i+1) = Theta(i) - maxturnrate*dt;

    v(i+1) = v(i) + Thrust/(m+mfuel(i))*dt;
    
    mfuel(i+1) = mfuel(i) - 14.71*dt;
    
    i = i+1;
end

mfuel(end)
plot(Hor, Alt)

