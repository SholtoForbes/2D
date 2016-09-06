function [x,y,z,V] = thirdstagemanipulation(file)

ThirdStageData = dlmread(file);
ThirdStageData = sortrows(ThirdStageData);

Vvals = unique(ThirdStageData(:,1));
thetavals = unique(ThirdStageData(:,2));
vvals = unique(ThirdStageData(:,3));


x = [Vvals(1):(Vvals(2)-Vvals(1)):Vvals(end)];
y = [thetavals(1):(thetavals(2)-thetavals(1)):thetavals(end)];
z = [vvals(1):(vvals(2)-vvals(1)):vvals(end)];

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


