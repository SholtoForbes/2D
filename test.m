init = cputime;
for i = 0.01:0.0001:0.1
D= OutForce([i,i],[8,9],50000,5000,60, communicator, communicator_trim);
% D1= OutForce(i,8,50000,5000,60, communicator, communicator_trim);
% D2= OutForce(i,9,50000,5000,60, communicator, communicator_trim);
end
fin = cputime;
time = fin - init
D
% D1
% D2
