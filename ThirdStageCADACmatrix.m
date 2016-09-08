clear all
j = 1;
payload_matrix = [];
% for Alt = [20000:2000:27000 27100:100:36000 38000:2000:50000]
% for theta = 0.0:.1:3
% for v = [1500:200:2700 2725:25:2900 3000]

for Alt = [32250:500:36250]
for theta = 0:.1:3
for v = [2700:50:2950]

% for lat = -pi:pi/3:pi
% for head = 0:30:180

i=1;
AoA_temp = [];
ThirdStage_temp = [];
for AoA = 10:.1:15
AoA_temp(i) = AoA;
ThirdStage_temp(i) = thirdstagesingle(AoA, Alt, theta, v);
i=i+1;
end
[payload_matrix(j,5),index] = max(ThirdStage_temp);
payload_matrix(j,4) = AoA_temp(index);
payload_matrix(j,1) = Alt;
payload_matrix(j,2) = theta;
payload_matrix(j,3) = v;

j = j+1;
end 
end
end
% end
% end


dlmwrite('thirdstagealternateheights1.dat', payload_matrix,'delimiter','\t')
