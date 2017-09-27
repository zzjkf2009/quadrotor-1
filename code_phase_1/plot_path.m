%author: Zejiang Zeng
%date: 09/09/2017
function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

% CMSC 828T Proj 1 Phase 1

%% START YOUR CODE HERE %%
Shortest_path=path;
%map_grid=cell2mat(map(1));
xy_res=cell2mat(map(2));
z_res=cell2mat(map(3));
%map_lowerC=cell2mat(map(4));
shifted_map=cell2mat(map(5));
BlockSize=cell2mat(map(6));

x0=shifted_map(1,1); x1=shifted_map(1,4);
y0=shifted_map(1,2); y1=shifted_map(1,5);
z0=shifted_map(1,3); z1=shifted_map(1,6);

vert=[x0 y0 z0;x1 y0 z0;x1 y1 z0;x0 y1 z0;x0 y0 z1;x1 y0 z1; x1 y1 z1;x0 y1 z1];
fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
patch('Vertices',vert,'Faces',fac,...
      'EdgeColor','blue','FaceColor','none','LineWidth',2);

  view(3)
axis equal
hold on

for l=2:BlockSize
x0=shifted_map(l,1); x1=shifted_map(l,4);
y0=shifted_map(l,2); y1=shifted_map(l,5);
z0=shifted_map(l,3); z1=shifted_map(l,6);
vert=[x0 y0 z0;x1 y0 z0;x1 y1 z0;x0 y1 z0;x0 y0 z1;x1 y0 z1; x1 y1 z1;x0 y1 z1];
%fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
patch('Vertices',vert,'Faces',fac,...
      'EdgeColor','none','FaceColor','red','LineWidth',2);

view(3)
axis equal
hold on
end

[stage,~]=size(Shortest_path);
for p=1:stage
x0=(Shortest_path(p,2)-1)*xy_res; x1=Shortest_path(p,2)*xy_res;
y0=shifted_map(1,5)-Shortest_path(p,1)*xy_res;y1=shifted_map(1,5)-(Shortest_path(p,1)-1)*xy_res;
z0=(Shortest_path(p,3)-1)*z_res; z1=Shortest_path(p,3)*z_res;
vert=[x0 y0 z0;x1 y0 z0;x1 y1 z0;x0 y1 z0;x0 y0 z1;x1 y0 z1; x1 y1 z1;x0 y1 z1];
%fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
patch('Vertices',vert,'Faces',fac,...
      'EdgeColor','none','FaceColor','yellow','LineWidth',2);
view(3)
axis equal
hold on   
    
end    
%% END YOUR CODE HERE %%

end