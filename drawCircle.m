% Philip Twu and Magnus Egerstedt
% Fall 2010
% © 2010 by Georgia Institute of Technology. All rights reserved.
%
% function drawCircle(c,r,opt,lineWidth)
% @param c (2 x 1) vector specifying the center of the circle
% @param r A positive scalar giving the radius of the circle
% @param opt A string giving the drawing options of the circle
% @param lineWidth The width of the line (circle's perimeter)
%
% This function draws a circle centered at 'c', with radius 'r', using
% plotting options 'opt', and line width 'lineWidth'

function drawCircle(c,r,opt,lineWidth)

P = 100; % Number of pts to use when drawing the circle

% Sample P pts in the circle of radius 'r' centered at 'c'
i = 0:P; % Row vector
pts = [c(1) + r*cos(2*pi*i./P); c(2) + r*sin(2*pi*i./P)]; % [2 x P+1 matrix]
 
% Plot the resulting circle
plot(pts(1,:),pts(2,:),opt,'LineWidth',lineWidth)