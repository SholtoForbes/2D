function endpointFunction = Brac1Events(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint function for the Brac: 1 Problem 
% Template for a A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

V0 = primal.states(1,1);		Vf = primal.states(1,end);
v0 = primal.states(2,1);		vf = primal.states(2,end);
theta0 = primal.states(3,1); thetaf = primal.states(3,end);
mfuel0 = primal.states(4,1);		mfuelf = primal.states(4,end);


% preallocate the endpointFunction evaluation for good MATLAB computing

endpointFunction = zeros(6,1); % t0 is specified in the problem file

%===========================================================
endpointFunction(1) = V0;
endpointFunction(2) = v0;
endpointFunction(3) = mfuel0;

%-----------------------------------------------------------
endpointFunction(4) = Vf;
endpointFunction(5) = thetaf;
endpointFunction(6) = mfuelf;

%-----------------------------------------------------------