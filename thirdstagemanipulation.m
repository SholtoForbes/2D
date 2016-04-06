function [x,y,z,V] = thirdstagemanipulation()

ThirdStageData = dlmread('thirdstage -old.dat');
ThirdStageData = sortrows(ThirdStageData);
x = [20000:2500:40000];
y = [-6:10];
z = [2000:200:3200];

n = 1;
for i = 1:9
    for j = 1:17
        for k = 1:7
            if ThirdStageData(n,1) == x(i) && ThirdStageData(n,2) == y(j) && ThirdStageData(n,3) == z(k)
            V(j,i,k) = ThirdStageData(n,5);
            n = n+1; 
            else
            V(j,i,k) = 0;

            end
               
        end
    end
end


