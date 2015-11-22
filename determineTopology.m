% Philip Twu and Magnus Egerstedt
% Fall 2010
% © 2010 by Georgia Institute of Technology. All rights reserved.
%
% function [A] = determineTopology(x,delta)
% @param x (N x 2) matrix of agent states, where row i gives the
%                  [x-position y-position] of agent i
% @param delta Two agents have information flow if and only if their
%              positions are less than or equal to delta from another
% @return A (N x N) adjacency matrix where A(i,j) = 1 if and only if
%           information flows between agents i and j 
%
% Takes in agent state information and returns the current information flow
% topology given by a delta-disk proximity graph.

function [A] = determineTopology(x,delta)

% Preallocate the adjacency matrix
N = size(x,1);
A = zeros(N);

for i=1:N-1
    for j=i+1:N
        % Two agents are neighbors iff they are within delta of each other
        if(norm(x(i,1:2) - x(j,1:2)) <= delta)
           A(i,j) = 1; 
           A(j,i) = 1;
        end
    end
end