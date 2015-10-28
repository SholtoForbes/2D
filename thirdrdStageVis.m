% visualising third stage cost results
clear all
ThirdStageData = dlmread('thirdstage.dat');
% map = [0, 0, 0
%     0, 0, 0
%     .0, 0, 0
%     .2, 0, 0
%     .6, 0, 0.0
%     1.0, 0, 0];
% colormap(map)

colormap jet
scatter3(ThirdStageData(:,2),ThirdStageData(:,3),ThirdStageData(:,1),30,ThirdStageData(:,4))





% j=1;
% k=0;
% for i = 1:length(ThirdStageData)
%     
% if ThirdStageData(i,3) == 2000
%     if ThirdStageData(i,2) == -0.2
%             k=k+1;
%             j = 1;
%     end
%     
%     Alt1(j,k) = ThirdStageData(i,1);
%     Theta1(j,k) = ThirdStageData(i,2);
%     Payload1(j,k) = ThirdStageData(i,4);
%     
%     
%     
%     j=j+1;
%     
%    
% 
% end
%     
% end
% 
% j=1;
% k=0;
% for i = 1:length(ThirdStageData)
%     
% if ThirdStageData(i,3) == 2500
%     if ThirdStageData(i,2) == -0.2
%             k=k+1;
%             j = 1;
%     end
%     
%     Alt2(j,k) = ThirdStageData(i,1);
%     Theta2(j,k) = ThirdStageData(i,2);
%     Payload2(j,k) = ThirdStageData(i,4);
%     
%     
%     
%     j=j+1;
%     
%    
% 
% end
%     
% end
% 
% j=1;
% k=0;
% for i = 1:length(ThirdStageData)
%     
% if ThirdStageData(i,3) == 3000
%     if ThirdStageData(i,2) == -0.2
%             k=k+1;
%             j = 1;
%     end
%     
%     Alt3(j,k) = ThirdStageData(i,1);
%     Theta3(j,k) = ThirdStageData(i,2);
%     Payload3(j,k) = ThirdStageData(i,4);
%     
%     
%     
%     j=j+1;
%     
%    
% 
% end
%     
% end
% 
% figure(1)
% surf(Alt1,Theta1,Payload1)
% 
% figure(2)
% surf(Alt2,Theta2,Payload2)
% 
% figure(3)
% surf(Alt3,Theta3,Payload3)




