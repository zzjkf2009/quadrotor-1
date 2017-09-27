%author: Zejiang Zeng
%date: 09/09/2017
function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

% CMSC 828T Proj 1 Phase 1

% If the map is empty, output empty vector else, compute
if isempty(map)
    C = [];
else
    %% START YOUR CODE HERE %%
[M,~]=size(points);
map_grid=cell2mat(map(1));
for i=1:M
    if map_grid(point(i,1),point(i,2),point(i,3))==1
        C(i)=1
    else 
        C(i)=0;
    end
end
    %% END YOUR CODE HERE %%
end
end
