% Philip Twu and Magnus Egerstedt
% Fall 2010
% © 2010 by Georgia Institute of Technology. All rights reserved.
%
% function [cleared] = checkFormation(x,formationCoords,agentRadius)
% @param x (N x 2) matrix of agent states, where row i gives the
%                  [x-position y-position] of agent i
% @param formationCoords A (M x 2) matrix of target points for the
%                        formation to be checked against
% @param agentRadius The radius of an agent.  
% @return cleared Takes on value 1 only if every target point in the
%                 formation has an agent that is touching it.
%
% Takes in the agent states and the target points for the formation.
% Checks to see whether the formation is attained by verifying that for
% each of the formation target points, there is an agent whose frame is
% touching it.

function [cleared] = checkFormation(x,formationCoords,agentRadius)

% Find number of agents
N = size(x,1);

% Find number of target points
M = size(formationCoords,1);

% Used to keep track of which target pts are met
formationMet = zeros(M,1);

for i=1:N % For each agent   
    for j=1:M % For each formation coord
       if(norm(x(i,:) - formationCoords(j,:)) <= agentRadius)
          formationMet(j) = 1; 
       end
   end
end

% Formation is satisfied if there is an agent touching each target
cleared = sum(formationMet) == M;