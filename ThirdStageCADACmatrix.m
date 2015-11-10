clear all
j = 1;
payload_matrix = [];
for Alt = 20000:2500:40000
for theta = 0:1:10
for v = 2000:200:3200

i=1;
AoA_temp = [];
ThirdStage_temp = [];
for AoA = 0:1:35
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


dlmwrite('thirdstage.dat', payload_matrix,'delimiter','\t')
