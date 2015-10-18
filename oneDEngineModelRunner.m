clear all
options = odeset('MaxStep',0.0001);
[x,prop] = ode45(@oneDEngineModel,[0 3],[300 2000 .01 1000], options);


hold on
plot(x,prop(:,1))
plot(x,prop(:,2))
plot(x,prop(:,3))
plot(x,prop(:,4))

ax1 = gca; % current axes
legend(ax1,  'T', 'U', 'rho', 'P')