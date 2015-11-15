% visualising third stage cost results
clear all
ThirdStageData = sortrows(dlmread('thirdstage.dat'));
% map = [0, 0, 0
%     0, 0, 0
%     .0, 0, 0
%     .2, 0, 0
%     .6, 0, 0.0
%     1.0, 0, 0];
% colormap(map)

% colormap gray
scatter3(ThirdStageData(:,2),ThirdStageData(:,3),ThirdStageData(:,1),30,ThirdStageData(:,5), 'filled')

xlabel('Trajectory Angle (deg)')
ylabel('Velocity (m/s)')
zlabel('Altitude (m)')



% for deleting large values

% for i = 1:length(ThirdStageData)
% 
% if (ThirdStageData(i,5) > 350 ) & (ThirdStageData(i,2) < 0)
%     
% ThirdStageData(i,:) = [];
%     
% end
% 
% end
% 
% 
% dlmwrite('thirdstage.dat', ThirdStageData,'delimiter','\t')
% j = 2;
% x = 1;
% y = 1;
% z = 1;
% %for creating a 3D array
% for i = 1:length(ThirdStageData)
% if ThirdStageData(i,2) >= 0
%     
% if (ThirdStageData(j,1) == ThirdStageData(j-1,1)) && (ThirdStageData(j,2) == ThirdStageData(j-1,2)) ;
%     
% threeDMAT(x,y,z) = ThirdStageData(j-1,5);
% threeDAlt(x,y,z) = ThirdStageData(j-1,1);
% threeDAngle(x,y,z) = ThirdStageData(j-1,2);
% threeDVel(x,y,z) = ThirdStageData(j-1,3);
% 
% x = x + 1;
% 
% 
% 
% elseif (ThirdStageData(j,1) == ThirdStageData(j-1,1))
%     
% threeDMAT(x,y,z) = ThirdStageData(j-1,5);
% threeDAlt(x,y,z) = ThirdStageData(j-1,1);
% threeDAngle(x,y,z) = ThirdStageData(j-1,2);
% threeDVel(x,y,z) = ThirdStageData(j-1,3);
% 
% x = 1;
% y = y+1;
% 
% else
%     
% threeDMAT(x,y,z) = ThirdStageData(j-1,5);
% threeDAlt(x,y,z) = ThirdStageData(j-1,1);
% threeDAngle(x,y,z) = ThirdStageData(j-1,2);
% threeDVel(x,y,z) = ThirdStageData(j-1,3);
% 
% x = 1;
% y = 1;
% z = z + 1;
% end
% j = j+1;
% end
% end
% 
% 
% Alt = [20000:2500:40000];
% Angle = [0:10];
% Vel = [2000:200:3200];
% 
% [meshAlt,meshAngle,meshVel] = meshgrid(Alt,Angle,Vel);
% 
% 
% % slice(threeDMAT,[1,3,5],[],[])
% % colormap hsv
% 
% 
% contourslice(threeDMAT,[1,3,6],[],[],10)
