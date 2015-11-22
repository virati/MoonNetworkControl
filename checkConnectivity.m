% Philip Twu and Magnus Egerstedt
% Fall 2010
% © 2010 by Georgia Institute of Technology. All rights reserved.
%
% function [isConnected] = checkConnectivity(A)
% @param A (N x N) adj matrix where A(i,j) = 1 iff info flows between i & j
% @return isConnected Takes on value 1 only if network is a connected graph
%
% Takes in the adjacency matrix of a network and checks whether or not it
% forms a connected graph by calculating the graph Laplacian and checking
% the second smallest eigenvalue.

function [isConnected] = checkConnectivity(A)

% Calculate the D matrix
D = diag(sum(A));

% Form the Laplacian
L = D - A;

% Compute the eigenvalues of the graph Laplacian
lambdas = eig(L);
lambdas = sort(lambdas);

% The network is a connected graph if and only if 2nd smallest eig is > 0
isConnected = (lambdas(2) > 1e-4);