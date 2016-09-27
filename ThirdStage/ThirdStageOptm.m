clear all
mScale = .1; % This needs to be manually changed in altitude and velocity files as well
AScale = 100;
x0 = [deg2rad(16)*AScale; 1300*mScale]
options.Algorithm = 'sqp'

options.Display = 'iter';
options.UseParallel = true;

% options.TypicalX = [10 10 10 1200];

% x = fmincon(@Velocity,x0,[],[],[],[],[13 13 13 1000],[17 17 17 1450],@Altitude,options)
x = fmincon(@Velocity,x0,[],[],[],[],[deg2rad(10)*AScale  1200*mScale],[deg2rad(17)*AScale  1500*mScale],[],options);

AoA_list = x(1:end-1)/AScale ;
mfuel_burn = x(end)/mScale;

rad2deg(AoA_list)
mfuel_burn

[AltF, vF, Alt, v, t, mpayload, Alpha] = ThirdStageSim(AoA_list, mfuel_burn);


figure(5)
plot(t, Alt)
figure(6)
plot(t,v)



% x0 = [1250*mscale]
% x = fmincon(@Velocity,x0,[],[],[],[],[1000*mscale],[1300*mscale],[],options)

% [AltF, vF] = ThirdStageSim([16 1250])


% for i = [1100:1200]
%     Velocity(i)
% end