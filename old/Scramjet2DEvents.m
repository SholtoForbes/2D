function endpointFunction = Brac1Events(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint function for the Brac: 1 Problem 
% Template for a A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%These can be anything, and define the layout of the bounds function in the
%problem file


H0 = primal.states(1,1);		Hf = primal.states(1,end);
V0 = primal.states(2,1);		Vf = primal.states(2,end);
% vh0 = primal.states(3,1);		vhf = primal.states(3,end);
% vv0 = primal.states(4,1);		vvf = primal.states(4,end);
theta0 = primal.states(3,1) ; thetaf = primal.states(3,end);
omega0 = primal.states(4,1) ; omegaf = primal.states(4,end); 

% preallocate the endpointFunction evaluation for good MATLAB computing
% endpointFunction = zeros(4,1);
endpointFunction = zeros(8,1);
% endpointFunction = zeros(8,1); % t0 is specified in the problem file
%need to change this depending on boundary conditions
%vx still being bound to 0 at endpoint, why??


% % ===========================================================
% endpointFunction(1) = x0;
% endpointFunction(2) = y0;
% endpointFunction(3) = vx0;
% endpointFunction(4) = vy0;
% 
% % -----------------------------------------------------------
% endpointFunction(5) = xf;
% endpointFunction(6) = yf; 
%  endpointFunction(7) = vxf;
%  endpointFunction(8) = vyf; 
% % -----------------------------------------------------------
% 


%===========================================================
endpointFunction(1) = H0;
endpointFunction(2) = V0;
endpointFunction(3) = theta0;
endpointFunction(4) = omega0;
% endpointFunction(3) = vh0;
% endpointFunction(4) = vv0;

%-----------------------------------------------------------
endpointFunction(5) = Hf;
endpointFunction(6) = Vf; 
endpointFunction(7) = thetaf;
endpointFunction(8) = omegaf;
%  endpointFunction(7) = vhf;
%  endpointFunction(8) = vvf; 
%-----------------------------------------------------------