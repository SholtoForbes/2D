function ThirdStageVisTrajectory(AoA, Alt, theta, v)
% Visualises a single third stage simulation

thirdstagesingle(AoA, Alt, theta, v);

ThirdStageData = dlmread('thirdstage.dat');

Traj = txt2mat('TRAJ3.ASC');

set(gcf,'position',[300 300 800 600])
hold on


plot(Traj(2:end-1,1),Traj(2:end-1,11)/1000, 'LineStyle', '-','Color','k', 'lineWidth', 2.0) % trajectory




xlabel('time (s)')
plot(Traj(2:end-1,1),Traj(2:end-1,8), 'LineStyle', '-','Color',[.5,.5,.5], 'lineWidth', 2.0) % trajectory angle
plot(Traj(2:end-1,1),Traj(2:end-1,5)/100, 'LineStyle', ':','Color','k', 'lineWidth', 2.0) % mass

plot(Traj(2:end-1,1),Traj(2:end-1,12)/100, 'LineStyle', '--','Color','k', 'lineWidth', 2.0) % velocity

plot(Traj(2:end-1,1),Traj(2:end-1,3)/1000, 'LineStyle', '-.','Color','k', 'lineWidth', 2.0) %dynamic pressure

legend(  'Altitude (km)', 'Trajectory Angle (degrees)', 'Mass (kg x 10^2)', 'Velocity (m/s x 10^2)', 'Dynamic Pressure (kPa)')

axis([0,Traj(end-1,1),0,Traj(end-1,11)/1000])
