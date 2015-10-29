clear all
j = 1;
for Alt = 20000:5000:50000
for theta = -3:0.5:6
for v = 2000:200:3200

i=1;
AoA_temp = [];
ThirdStage_temp = [];
for AoA = 5:0.5:15
AoA_temp(i) = AoA;
ThirdStage_temp(i) = thirdstagesingle(AoA, Alt, theta, v);
i=i+1;
end
[payload_max(j),index] = max(ThirdStage_temp);
AoA_max(j) = AoA_temp(index);
Alt_list(j) = Alt;
theta_list(j) = theta;
v_list(j) = v;

j = j+1;
end 
end
end