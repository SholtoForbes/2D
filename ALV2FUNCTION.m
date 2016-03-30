function [diff,t,r,gamma,v,m,xi,phi,zeta,alpha,beta] = ALV2FUNCTION(x,r0,xi0,phi0,zeta0,rTarget,gammaTarget)
gamma0 = x(1);


r_E = 6371000; % radius of Earth (m)

% ALV2 Simulation Function
% Sholto Forbes-Spyratos


mPayload = 8755.1; % SPARTAN mass
% mPayload = 450;

SCALE = 7.0; % Scaling of the ALV2


% Atmosphere Data (1976 NASA Model)
atmosphere = dlmread('atmosphere.txt');

% AeroCoefficients
% PLACEHOLDERS, NEED TO UPDATE THESE

AeroCoeffs = dlmread('AeroCoeffs.txt');
AeroCoeffs = [[0 0];AeroCoeffs];

% Thrust Libraries
% Altitude(km)  Pressure(atm)  Effective Exhaust Velocity(m/s)
% Specific impulse(s)  Thrust Coefficient  Thrust(kN)
FirstStageThrust = dlmread('FirstStageThrust.txt');


% no. First Stage Modules
N = 4;

A1 = sqrt(SCALE)*(.28 + N*0.5); % Reference Area of first stage with 4 boosters, each booster is 0.5 and core stage is 0.28 (m^2)

% Define First Stage Characteristics For 1 Module

mP1 = SCALE*1600; % First Stage Propellant Loading (Total) (kg)
PCR1 = SCALE*16.39; % First Stage Propellant Consumption Rate (Total) (kg/s)
mB1 = SCALE*480; % First Stage Burnout Mass (kg)
mAF = SCALE*70; % Aero Fuel (kg)
mL = SCALE*410; % Landing Mass (kg)

% Set Conditions
r_E = 6371000; % radius of Earth (m)
dt = 1.; % Timestep (s)
alphadt_max = .5; % maximum angle of attack change rate deg/s


%==========================================================================
%----------------------- First Stage Simulation ---------------------------
%==========================================================================

% Cartesian Coordinates

% L = 0; % Lift, none
alpha(1) = 0; % Angle of Attack, zero

% Initialise Arrays & Set Initial Conditions
% Launch Conditions
r(1) = r0 + r_E; % Radius (m)
xi(1) = xi0; % Longitude (rad)
phi(1) = phi0; % Latitude (rad)
gamma(1) = deg2rad(gamma0); % Flight Path Angle (rad)
v(1) = 0; % Velocity (m/s)
zeta(1) = zeta0; % Heading Angle (rad)

beta(1) = gamma(1); % Thrust Inclination (rad)

% Vehicle Conditions
m(1) = (mP1 + mAF + mL)*N  + mPayload ; % Mass Array (kg)
t(1) = 0; % Time Array (s)
v_a = interp1(atmosphere(:,1),atmosphere(:,5),r(1)-r_E); % Speed of Sound
M(1) = v(1)/v_a; % Mach no Array
Cd(1) = interp1(AeroCoeffs(:,1),AeroCoeffs(:,2),M(1)); % Drag Coefficient Array (N);
D(1) = 0.5* Cd * v(1)^2 * A1 * interp1(atmosphere(:,1),atmosphere(:,4),r(1)-r_E);

L(1) = D(1)*sin(alpha(1));
% L(1) = 0;

% SCALES THRUST, this is NOT REALISTIC (scaling should be sqrt(scale)).
% This is to keep the thrust able to lift the payload
T(1) = 1.0*SCALE*interp1(FirstStageThrust(:,1),FirstStageThrust(:,6),0)*N*3  * 1000; % Thrust Array (kN) (3 Engines on Each ALV-2 First Stage Module)

mParray(1) = mP1*N; % Initialise Propellant Mass Array (kg)

tBurnout = mP1/PCR1;

i = 1; % Temporary Iteration Counter

section = '1';

t_pitch = 20; % pitchover time

while mParray(i) > 0 && r(i) >= r_E

% Increment Equations of Motion

[rdot,xidot,phidot,gammadot,vdot,zetadot] = RotCoords(r(i),xi(i),phi(i),gamma(i),v(i),zeta(i),L(i),D(i),T(i),m(i),alpha(i));

r(i+1) = r(i) + dt*rdot;
xi(i+1) = xi(i) + dt*xidot;
phi(i+1) = phi(i) + dt*phidot;
gamma(i+1) = gamma(i) + dt*gammadot;
v(i+1) = v(i) + dt*vdot;
zeta(i+1) = zeta(i) + dt*zetadot;

% Initiate Turn, prevent gamma from changing before pitchover time
if t(i) < t_pitch
gamma(i+1) = gamma(i);
zeta(i+1) = zeta(i);
beta(i+1) = beta(i);
alpha(i+1) = 0;

elseif t(i) >= t_pitch && strcmp(section,'1') == 1 % Pitchover Time (Arbitrary Currently)
gamma(i+1) = gamma(i)-deg2rad(1); % Pitchover Angle of 1 Degree (assumed to be instantaneous)
zeta(i+1) = zeta(i);
section = '2';
% alpha(i+1) = deg2rad(alpha_p); % set angle of attack after turn start
beta(i+1) = beta(i)-deg2rad(1);
alpha(i+1) = 0;
t2 = t(i);
beta2 = beta(i+1);
elseif strcmp(section,'2') == 1
% alpha(i+1) = deg2rad(alpha_p) + atan(x(2)*(t(i)-t_pitch));

% beta(i+1) = beta2 + atan(x(2)*(t(i)-t2));
beta(i+1) = atan(tan(beta2)*(1 - (t(i)-t_pitch)/(tBurnout-t_pitch)));
alpha(i+1) = beta(i+1) - gamma(i+1);
end

    
% Increment Vehicle Parameters
    
mParray(i+1) = mParray(i) - PCR1*N*dt; 

m(i+1) = m(i) - PCR1*N*dt;

if r(i+1)-r_E < 48000
T(i+1) = 1.0*SCALE*interp1(FirstStageThrust(:,1),FirstStageThrust(:,6),(r(i+1)-r_E)/1000) * N*3 * 1000; 
else
T(i+1) = 1.0*SCALE*interp1(FirstStageThrust(:,1),FirstStageThrust(:,6),48000/1000) * N*3  * 1000;
end

if r(i+1)-r_E < 85000
v_a = interp1(atmosphere(:,1),atmosphere(:,5),r(i+1)-r_E);
else
v_a = interp1(atmosphere(:,1),atmosphere(:,5),85000);
end

M(i+1) = v(i+1)/v_a;

Cd(i+1) = interp1(AeroCoeffs(:,1),AeroCoeffs(:,2),M(i+1)) + 1.1*sin(alpha(i+1))^3; 

if r(i+1)-r_E < 85000
D(i+1) = 0.5 * Cd(i+1) * v(i+1)^2 * A1 * interp1(atmosphere(:,1),atmosphere(:,4),r(i+1)-r_E);
else
D(i+1) = 0;
end

L(i+1) = D(i+1)*sin(alpha(i+1));
% L(i+1) = 0;

t(i+1) = t(i) + dt;

i = i+1;
end
temp_1 = i;


i12 = i; % Node No Of Separation


diff = abs(r(end) - r_E -rTarget) %- v(end)
% diff = 1000*abs(gamma(end) - gammaTarget) - v(end)
% diff = abs(r(end) - r_E -rTarget) + 1000*abs(gamma(end) - gammaTarget) %- v(end)
% figure
% plot(t,rad2deg(gamma))
% figure
% plot(t,r-r_E)
% figure
% plot(t,v)
end

