function endpointFunction = Brac1Events(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Event Function for 2D Problem
% Written by Sholto Forbes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global MultiStage


V0 = primal.states(1,1);        
H0 = primal.states(2,1);        
v0 = primal.states(3,1);        
vf = primal.states(3,end);

if MultiStage == 1
    Left    = primal.indices.left;      
    Right   = primal.indices.right;
    preseparation_v = primal.states(3,Left);
    postseparation_v = primal.states(3,Right);
    preseparation_V = primal.states(1,Left);
    postseparation_V = primal.states(1,Right);
    preseparation_H = primal.states(2,Left);
    postseparation_H = primal.states(2,Right);
end

% thetaf = primal.controls(end);
% endpointFunction = zeros(5,1);

if MultiStage == 1
    endpointFunction = zeros(8,1);
else
    endpointFunction = zeros(4,1);
end
%===========================================================
endpointFunction(1) = V0;

endpointFunction(2) = H0;

endpointFunction(3) = v0;
endpointFunction(4) = vf;

if MultiStage == 1
    endpointFunction(5) = preseparation_v;
    endpointFunction(6) = postseparation_v;
    endpointFunction(7) = postseparation_V - preseparation_V;
    endpointFunction(8) = postseparation_H - preseparation_H;
    
end
% endpointFunction(5) = thetaf;
%-----------------------------------------------------------




