function Q = Brac1Path(primal)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global heating_rate

time = primal.nodes;

dt_array = time(2:end)-time(1:end-1); % Time change between each node pt

Q = zeros(1,length(time));
Q(1) = 0;

for i = 1:length(dt_array)
    Q(i+1) = heating_rate(i)*dt_array(i);
end
