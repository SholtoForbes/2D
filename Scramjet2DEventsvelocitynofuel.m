function endpointFunction = Brac1Events(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint function for the Brac: 1 Problem 
% Template for a A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%These can be anything, and define the layout of the bounds function in the
%problem file

%with velocity constraints
% vH0 = primal.states(1,1);		vHf = primal.states(1,end);
% vV0 = primal.states(2,1);		vVf = primal.states(2,end);
% H0 = primal.states(4,1);        Hf = primal.states(4,end);

%no velocity constraints
H0 = primal.states(2,1);		Hf = primal.states(2,end);
v0 = primal.states(1,1);

% preallocate the endpointFunction evaluation for good MATLAB computing
% endpointFunction = zeros(6,1);
endpointFunction = zeros(3,1);

%need to change this depending on boundary conditions


%===========================================================
% endpointFunction(1) = vH0;
% endpointFunction(2) = vV0;
% endpointFunction(3) = H0;

endpointFunction(1) = H0;
endpointFunction(2) = Hf;
endpointFunction(3) = v0;


%-----------------------------------------------------------
% endpointFunction(4) = vHf;
% endpointFunction(5) = vVf; 
% endpointFunction(6) = Hf;

%-----------------------------------------------------------