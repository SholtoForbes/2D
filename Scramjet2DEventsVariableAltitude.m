function endpointFunction = Brac1Events(primal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Endpoint function for the Brac: 1 Problem 
% Template for a A Beginner's Guide to DIDO 
% I. Michael Ross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% AN EASY WAY TO SWITCH BETEEN MULTI AND SINGLE STAGE
global MultiStage


%These can be anything, and define the layout of the bounds function in the
%problem file

%with velocity constraints
% vH0 = primal.states(1,1);		vHf = primal.states(1,end);
% vV0 = primal.states(2,1);		vVf = primal.states(2,end);
% H0 = primal.states(4,1);        Hf = primal.states(4,end);

%no velocity constraints
V0 = primal.states(1,1);        Vf = primal.states(1,end);
H0 = primal.states(2,1);        Hf = primal.states(2,end);

% MULTI STAGE ===========================================================
if MultiStage ==1
% %--------------------------------------------------------------------------
% % Collect all the left and right limit points
% Left    = primal.indices.left;      
% Right   = primal.indices.right;
% %--------------------------------------------------------------------------
% % If this problem had more than two stages, Left and Right would be row
% % vectors; here they are just numbers
% %------------------------------------------------------------------------
% preSeparation_v = primal.states(1, Left);   postSeparation_v = primal.states(1, Right);
% preSeparation_H = primal.states(2, Left);   postSeparation_H = primal.states(2, Right);

end
%=========================================================================


% preallocate the endpointFunction evaluation for good MATLAB computing
if MultiStage ==1
%     endpointFunction = zeros(6,1); %MULTI STAGE
else
    endpointFunction = zeros(4,1);
end
%need to change this depending on boundary conditions


%===========================================================
% endpointFunction(1) = vH0;
% endpointFunction(2) = vV0;
% endpointFunction(3) = H0;
endpointFunction(1) = V0;
endpointFunction(2) = Vf;
endpointFunction(3) = H0;
endpointFunction(4) = Hf;


if MultiStage ==1
% endpointFunction(5) = preSeparation_v - postSeparation_v; % dv over stage transition
% endpointFunction(6) = preSeparation_H - postSeparation_H; % dH over stage transition
end
%-----------------------------------------------------------
% endpointFunction(4) = vHf;
% endpointFunction(5) = vVf; 
% endpointFunction(6) = Hf;

%-----------------------------------------------------------