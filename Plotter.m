clear all

'Select Primal File'
[file1 path] = uigetfile('*.txt','Select the text-file')
filestr1 = strcat(path,file1);
data1 = dlmread(filestr1) ;




V = data1(1,:);

v = data1(2,:);

theta = data1(3,:);

mfuel = data1(4,:);

thetadot = data1(5,:);

t = data1(6,:);


q = data1(7,:);

IspNet = data1(8,:);

Alpha = data1(9,:);


% figure out horizontal motion
H(1) = 0;
for i = 1:length(V)-1
H(i+1) = v(i)*(t(i+1) - t(i))*cos(theta(i)) + H(i);
end


H(1) = 0;
for i = 1:length(V)-1
H(i+1) = v(i)*(t(i+1) - t(i))*cos(theta(i)) + H(i);
end


ThirdStageData = dlmread('thirdstage.dat');
[alt_list,gamma_list,v_list,payload_array] = thirdstagemanipulation();
ThirdStagePayloadMass = interp3(alt_list,gamma_list,v_list,payload_array,V(end), rad2deg(theta(end)), v(end),'cubic');
ThirdStageAoASpline = scatteredInterpolant(ThirdStageData(:,1),ThirdStageData(:,2),ThirdStageData(:,3),ThirdStageData(:,4));
AoA = ThirdStageAoASpline(V(end), rad2deg(theta(end)), v(end));
figure(4)
evalc('ThirdStageVisTrajectory(AoA, V(end), rad2deg(theta(end)), v(end));');

figure(1)



subplot(2,6,[1,6])
title('Trajectory Comparison')
hold on
ax = gca;
plot(H/1000, V/1000, 'Color','k', 'LineStyle', '-')

h = legend(ax,  'Trajectory');

hlt = text(...
    'Parent', h.DecorationContainer, ...
    'String', 'Maximum Dynamic Pressure', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom', ...
    'Position', [0.5, 1.05, 0], ...
    'Units', 'normalized');

subplot(2,6,[7,9])
hold on
ax2 = gca;
plot(t, v/10^3, 'Color','k', 'LineStyle', '-')

plot(t, q/10^4, 'Color','k', 'LineStyle', '--')

h = legend(ax2,  'Velocity (m/s x 10^3)', 'Dynamic Pressure (kPa x 10^4)');


subplot(2,6,[10,12])
hold on
ax3 = gca;
plot(t, IspNet/10^2, 'Color','k', 'LineStyle', '-')

plot(t, Alpha, 'Color','k', 'LineStyle', '--')

h = legend(ax3,  'Net Isp (s x 10^2)', 'Angle of Attack (degrees)');

