% visualising third stage cost results
clear all
ThirdStageData = sortrows(dlmread('thirdstage.dat'));
Atmosphere = dlmread('atmosphere.txt');
% map = [0, 0, 0
%     0, 0, 0
%     .0, 0, 0
%     .2, 0, 0
%     .6, 0, 0.0
%     1.0, 0, 0];
% colormap(map)




colormap jet
scatter3(ThirdStageData(:,2),ThirdStageData(:,3),ThirdStageData(:,1)/1000,30,ThirdStageData(:,5), 'filled')

xlabel('Trajectory Angle (deg)')
ylabel('Velocity (m/s)')
zlabel('Altitude (km)')
xlim([-6,10])
c=colorbar('northoutside');
ylabel(c,'Payload Mass, kg')



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



j = 2;
x = 1;
y = 1;
z = 1;

%for creating a 3D array
for i = 1:length(ThirdStageData)-1

    
if ThirdStageData(i,2) >= 0 && (ThirdStageData(j,1) == ThirdStageData(j-1,1)) && (ThirdStageData(j,2) == ThirdStageData(j-1,2)) ;
    
threeDMAT(x,y,z) = ThirdStageData(j-1,5);
threeDAlt(x,y,z) = ThirdStageData(j-1,1);
threeDAngle(x,y,z) = ThirdStageData(j-1,2);
threeDVel(x,y,z) = ThirdStageData(j-1,3);


x = x + 1;



elseif ThirdStageData(i,2) >= 0 && (ThirdStageData(j,1) == ThirdStageData(j-1,1))
    
threeDMAT(x,y,z) = ThirdStageData(j-1,5);
threeDAlt(x,y,z) = ThirdStageData(j-1,1);
threeDAngle(x,y,z) = ThirdStageData(j-1,2);
threeDVel(x,y,z) = ThirdStageData(j-1,3);

x = 1;
y = y+1;

elseif ThirdStageData(i,2) >= 0
    
threeDMAT(x,y,z) = ThirdStageData(j-1,5);
threeDAlt(x,y,z) = ThirdStageData(j-1,1);
threeDAngle(x,y,z) = ThirdStageData(j-1,2);
threeDVel(x,y,z) = ThirdStageData(j-1,3);

x = 1;
y = 1;
z = z + 1;
else
    
end

j = j+1;

end


Alt = [20000:2500:40000];
Angle = [0:10];
Vel = [2000:200:3200];

[meshAlt,meshAngle,meshVel] = meshgrid(Alt,Angle,Vel);



% creating a 2d grid at 2800m/s

x = 1;
y = 1;
jprev = 20000;


for j = 20000:2500:40000

for k = 0:10

for i = 1:length(ThirdStageData)
    
if ThirdStageData(i,1) == j && ThirdStageData(i,2) == k && ThirdStageData(i,3) == 2800 
    
meshPayload(x,y) =   ThirdStageData(i,5) ;

end

end

% if j == jprev
% y = y+1;
% else
% x = x+1; 
% y = 1;
% end

x = x+1;

end
x=1;
y = y+1;
end

figure(2)
colormap(jet)
contourf(meshAngle(:,:,5),meshAlt(:,:,5)/1000,meshPayload,13)
xlabel('Trajectory Angle (deg)')
ylabel('Altitude (km)')
c=colorbar('northoutside')
ylabel(c,'Payload Mass, kg')






% slice(threeDMAT,[1,3,5],[],[])
% colormap hsv


% contourslice(threeDMAT,[1,3,6],[],[],10)



% getting maximum values
[max1,ind1] = max(threeDMAT,[],3);

[max2,ind2] = max(max1,[],2);

ind  = [ind1(ind2),ind2] % this really doesnt tell me much, as it wants very high altitude all the time





% plotting density


figure(3)
plot(Atmosphere(:,1)/1000,  Atmosphere(:,4)),'Color','k','LineWidth',0.8;
xlim([20,40])

xlabel('Altitude (km)')
ylabel('Density (kg/m^3)')



