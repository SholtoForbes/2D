function endpointFunction = Brac1Events(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Event Function for 2D Problem
% Written by Sholto Forbes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Stage
global const
     


v0 = primal.states(2,1);  

 

mfuel0 = primal.states(4,1);


mfuelf = primal.states(4,end);



if const == 1
% 
% endpointFunction = zeros(3,1);
endpointFunction = zeros(2,1);
end

if const == 2
Q0 = primal.states(5,1);

Qf = primal.states(5,end);
    
endpointFunction = zeros(4,1);
end

if const == 3
endpointFunction = zeros(3,1);
vf = primal.states(2,end);
end

if const == 4
endpointFunction = zeros(5,1);
vf = primal.states(2,end);
V0 = primal.states(1,1); 
q0 = primal.states(5,1);
end

%===========================================================

endpointFunction(1) = v0;

if const == 1
% 

endpointFunction(2) = mfuel0;
% endpointFunction(3) = mfuelf;
end

if const == 2
endpointFunction(2) = mfuel0;
endpointFunction(3) = Q0;
endpointFunction(4) = Qf;
end


if const == 3
endpointFunction(2) = vf;
endpointFunction(3) = mfuel0;
end


if const == 4

    
endpointFunction(2) = vf;
endpointFunction(3) = mfuel0;
endpointFunction(4) = V0;
endpointFunction(5) = q0;
end
