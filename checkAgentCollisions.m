% Philip Twu and Magnus Egerstedt
% Fall 2010
% © 2010 by Georgia Institute of Technology. All rights reserved.
%
% function [agentCol] = checkAgentCollisions(x,agentRadius)
% @param x (N x 2) matrix of agent states, where row i gives the
%                  [x-position y-position] of agent i
% @param agentRadius The radius of an agent.  Two agents collide if they
%                    are within 2*agentRadius of one another and an agent 
%                    collides with an obstacles if the obstacle is within 
%                    agentradius of it.
% @return agentCol Takes on value 1 only if agents collided with each other
%
% Takes in information about the agent states and their radius.  From the
% obstacle map and agent positions, determines whether agents collide.
function [agentCol] = checkAgentCollisions(x,agentRadius)

% Implicitly figure out N from the state vector
N = size(x,1);

% Check to see if the agents are colliding with each other
agentCol = 0;
for i=1:N-1
   for j=i+1:N
       % Two agents collide if centroids are within 2*agentRadius 
       if(norm(x(i,1:2) - x(j,1:2)) <= 2*agentRadius)
          agentCol = 1;
       end
   end
end