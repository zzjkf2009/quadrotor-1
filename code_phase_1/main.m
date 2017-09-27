%author: Zejiang Zeng
%date: 09/09/2017
clear
tic
filename='sample_maps/map0.txt';
xy_res=0.8;
z_res=2;
start=[4,-2,4.4];
stop=[8.2,18.5,3.0];
map = load_map(filename,0.15, 0.56, 0.2);
%C = collide(map, points);
tic;
[path, num_expanded] = dijkstra(map, start, stop);
 toc;
plot_path(map, path)
