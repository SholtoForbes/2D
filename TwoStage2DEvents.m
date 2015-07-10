function endpointFunction = Brac1Events(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint function for the Brac: 1 Problem 
% Template for a A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

V0 = primal.states(1,1);        %Vf = primal.states(1,end);
H0 = primal.states(2,1);        %Hf = primal.states(2,end);

%velocity primal
v0 = primal.states(3,1);        vf = primal.states(3,end);



% endpointFunction = zeros(4,1);
% %===========================================================
% endpointFunction(1) = V0;
% endpointFunction(2) = Vf;
% endpointFunction(3) = H0;
% endpointFunction(4) = Hf;
% %-----------------------------------------------------------

endpointFunction = zeros(4,1);
%===========================================================
endpointFunction(1) = V0;
% endpointFunction(2) = Vf;
endpointFunction(2) = H0;
% endpointFunction(4) = Hf;
endpointFunction(3) = v0;
endpointFunction(4) = vf;
%-----------------------------------------------------------




