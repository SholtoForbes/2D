clear all
j = 1;
payload_matrix = [];
for Alt = 34500:100:35500
for theta = 2.5:.1:3
for v = 2700:50:2900

i=1;
AoA_temp = [];
ThirdStage_temp = [];
for AoA = 0:1:20
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


dlmwrite('thirdstagearound35km.dat', payload_matrix,'delimiter','\t')
