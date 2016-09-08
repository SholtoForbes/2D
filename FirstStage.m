% ALV2 Velocity Calculator
clear all

rTarget = 27000;
gammaTarget = 0;


r0 = 0;
xi0 = 0;
phi0 = 0;
zeta0 = 0;


% optimisation parameters
A = [];
b = [];
Aeq = [];
beq = [];
lb = [0]; % lower bounds of A, B
ub = [90]; % upper bounds of A, B

x0 = [0];

nonlcon = [];
options = optimoptions('fmincon','Display','iter','Algorithm','sqp','UseParallel',true);
options.MaxFunEvals = 400 ;
% 
x = fmincon(@(x)ALV2FUNCTION(x,r0,xi0,phi0,zeta0,rTarget,gammaTarget),x0,A,b,Aeq,beq,lb,ub,nonlcon, options) 

[diff,t,r,gamma,v,m,xi,phi,zeta,alpha,beta,T] = ALV2FUNCTION(x,r0,xi0,phi0,zeta0,rTarget,gammaTarget);


r_E = 6371000; % radius of Earth (m)
figure
hold on
plot(t,rad2deg(gamma))
plot(t,rad2deg(beta))
plot(t,rad2deg(alpha))
figure
plot(t,r-r_E)
figure
plot(t,v)


