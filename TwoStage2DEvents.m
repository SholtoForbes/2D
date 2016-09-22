function endpointFunction = TwoStage2DEvents(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Event Function for 2D Problem
% Written by Sholto Forbes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Stage
global const
     

V0 = primal.states(1,1); 
Vf = primal.states(1,end); 
v0 = primal.states(2,1);  
gamma0 = primal.states(3,1);  
mfuel0 = primal.states(4,1);
mfuelf = primal.states(4,end);

if const == 1 || const == 12 || const == 14
endpointFunction = zeros(3,1); % previous, working
end

if const == 13
endpointFunction = zeros(4,1); 
% endpointFunction = zeros(3,1); 
end

if const == 3
endpointFunction = zeros(3,1);
end

%===========================================================

endpointFunction(1) = v0;

if const == 1 || const == 12 || const == 13 || const == 14
endpointFunction(2) = mfuel0;
endpointFunction(3) = mfuelf;
end

if const == 13
endpointFunction(4) = V0;
end

if const == 3
endpointFunction(2) = mfuel0;
endpointFunction(3) = mfuelf;
end

end
