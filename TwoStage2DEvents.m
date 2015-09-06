function endpointFunction = Brac1Events(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Event Function for 2D Problem
% Written by Sholto Forbes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Stage


V0 = primal.states(1,1);        
% H0 = primal.states(2,1);        
% v0 = primal.states(3,1);        
% vf = primal.states(3,end);

v0 = primal.states(2,1);        
vf = primal.states(2,end);

mfuel0 = primal.states(4,1);
% mfuel0 = primal.states(3,1);

% thetaf = primal.controls(end);
% endpointFunction = zeros(5,1);

% endpointFunction = zeros(4,1);
endpointFunction = zeros(3,1);
% endpointFunction = zeros(2,1);

%===========================================================
% endpointFunction(1) = V0;
% 
% endpointFunction(2) = H0;
% 
% endpointFunction(3) = v0;
% endpointFunction(4) = vf;
% 
% % endpointFunction(5) = thetaf;
%-----------------------------------------------------------
% endpointFunction(1) = V0;
% endpointFunction(2) = v0;
% endpointFunction(3) = vf;
% endpointFunction(4) = mfuel0;

endpointFunction(1) = v0;
endpointFunction(2) = vf;
endpointFunction(3) = mfuel0;




