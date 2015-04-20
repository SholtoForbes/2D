function endpointFunction = Brac1Events(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint function for the Brac: 1 Problem 
% Template for a A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x0 = primal.states(1,1);		xf = primal.states(1,end);
y0 = primal.states(2,1);		yf = primal.states(2,end);
vx0 = primal.states(3,1);		vxf = primal.states(3,end);
vy0 = primal.states(4,1);		vyf = primal.states(4,end);

% preallocate the endpointFunction evaluation for good MATLAB computing
endpointFunction = zeros(4,1);
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
endpointFunction(1) = x0;
endpointFunction(2) = y0;
% endpointFunction(3) = vx0;
% endpointFunction(4) = vy0;

%-----------------------------------------------------------
endpointFunction(3) = xf;
endpointFunction(4) = yf; 
%  endpointFunction(7) = vxf;
%  endpointFunction(8) = vyf; 
%-----------------------------------------------------------