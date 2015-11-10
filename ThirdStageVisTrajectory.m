function ThirdStageVisTrajectory(AoA, Alt, theta, v)
% Visualises a single third stage simulation

thirdstagesingle(AoA, Alt, theta, v);

ThirdStageData = dlmread('thirdstage.dat');

Traj = txt2mat('TRAJ3.ASC');

hold on

subplot(2,5,[1,5])


plot(Traj(2:end-1,1),Traj(2:end-1,11), 'LineStyle', '-','Color','k', 'lineWidth', 2.0) % trajectory
ylabel('Altitude (m)')
xlabel('time (s)')

subplot(2,5,[6,10])
ax2 = gca;
xlabel('time (s)')
line(Traj(2:end-1,1),Traj(2:end-1,8), 'LineStyle', '-','Color','k', 'lineWidth', 2.0) % trajectory angle
line(Traj(2:end-1,1),Traj(2:end-1,5)/100, 'LineStyle', ':','Color','k', 'lineWidth', 2.0) % mass

line(Traj(2:end-1,1),Traj(2:end-1,12)/100, 'LineStyle', '--','Color','k', 'lineWidth', 2.0) % velocity

line(Traj(2:end-1,1),Traj(2:end-1,3)/1000, 'LineStyle', '-.','Color','k', 'lineWidth', 2.0) %dynamic pressure

legend(ax2,  'Trajectory Angle (degrees)', 'Mass (kg x 10^2)', 'Velocity (m/s x 10^2)', 'Dynamic Pressure (kPa)')


