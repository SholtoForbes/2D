function [x,y,z,V] = thirdstagemanipulation(file)

ThirdStageData = dlmread(file);
ThirdStageData = sortrows(ThirdStageData);

Vvals = unique(ThirdStageData(:,1));
thetavals = unique(ThirdStageData(:,2));
vvals = unique(ThirdStageData(:,3));

x = Vvals;
y = thetavals;
z = vvals;

n = 1;
for i = 1:length(Vvals)
    for j = 1:length(thetavals)
        for k = 1:length(vvals)

            if ThirdStageData(n,1) == x(i) && ThirdStageData(n,2) == y(j) && ThirdStageData(n,3) == z(k) % allows for null values in certain cells
            V(j,i,k) = ThirdStageData(n,5);
            n = n+1; 
            else
            V(j,i,k) = 0;

            end
               
        end
    end
end


