function Target = Velocity(x)
% AScale = 100;
mScale = .01;
% AoA_list = x(1:end-1)/AScale ;
mfuel_burn = x/mScale;

% mfuel_burn = x(1)/mscale

[AltF, vF, Alt, v, t, mpayload, Alpha] = ThirdStageSim(mfuel_burn);
% [AltF, vF] = ThirdStageSim(15, mfuel_burn);

x;
% Target = -vF;
Target = -mpayload;
end