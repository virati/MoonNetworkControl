% Philip Twu and Magnus Egerstedt
% Fall 2010
% © 2010 by Georgia Institute of Technology. All rights reserved.
%
% function [wpCleared] = checkClearWaypoint(x,wp,currWP,wpRadius,delta,numAgents)
% @param x (N x 2) matrix of agent states, where row i gives the
%                  [x-position y-position] of agent i
% @param wp (W x 2) matrix of waypoint locations, where row i gives the
%                  [x-position y-position] of wp i
% @param currWP The current waypoint that is to be cleared
% @param wpRadius The proximity in which an agent must get within a
%                 waypoint in order to clear it
% @param delta Two agents have information flow if and only if their
%              positions are less than or equal to delta from another
% @param numAgents The number of agents that are supposed to be in the
%                  network when the waypoint is cleared
% @return wpCleared Return value is 1 if a waypoint was cleared this round
%
% Takes in the agent states, list of waypoint locations, current waypoint,
% and minimum proximity required for clearing a waypoint.  Checks to see if
% the current waypoint is cleared and that no agents have been lost.

function [wpCleared] = checkClearWaypoint(x,wp,currWP,wpRadius,delta,numAgents)

% The waypoint is not cleared by default
wpCleared = 0;

% Determine the number of agents
N = size(x,1);

% First check to see if the number of agents in the network is correct
if(numAgents == N)
    % Clear a waypoint by letting an agent get within wpRadius of it
    for i=1:N
        if(norm(x(i,1:2) - wp(currWP,:)) <= wpRadius)
            wpCleared = 1;
            break;
        end
    end
end

% Graph must also be connected in order to clear the waypoint
wpCleared = wpCleared && checkConnectivity(determineTopology(x,delta));
