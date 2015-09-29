function endpointFunction = Brac1Events(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Event Function for 2D Problem
% Written by Sholto Forbes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Stage


V0 = primal.states(1,1);        

v0 = primal.states(2,1);        
vf = primal.states(2,end);

mfuel0 = primal.states(4,1);

% Q0 = primal.states(5,1);

endpointFunction = zeros(3,1);
% endpointFunction = zeros(4,1);

%===========================================================

endpointFunction(1) = v0;
endpointFunction(2) = vf;
endpointFunction(3) = mfuel0;
% endpointFunction(4) = Q0;




