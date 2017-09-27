%author: Zejiang Zeng
%date: 09/09/2017
function [Shortest_path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.

% CMSC 828T Proj 1 Phase 1

if nargin < 4
    astar = false;
end

%% START YOUR CODE HERE %%
% where map is a M-N-K matrix 
map_grid=cell2mat(map(1));
xy_res=cell2mat(map(2));
z_res=cell2mat(map(3));
map_lowerC=cell2mat(map(4));
[Height,Width,Thickness]=size(map_grid);
GScore=zeros(Height,Width,Thickness);   %Matrix keeping track of G-scores 
FScore=single(inf(Height,Width,Thickness));  %Matrix keeping track of F-scores (only open list) 
%HScore=single(zeros(Height,Width,Thickness));   %Heuristic matrix
%OpenMAT=false(Height,Width,Thickness); %Matrix keeping of open grid cells
ClosedMAT=false(Height,Width,Thickness);   %Matrix keeping track of closed grid cells
ClosedMAT(map_grid==1)=1;  %Adding object-cells to closed matrix
Parent=cell(Height,Width,Thickness); %store parent node into cells
%%%%%%%%%%%%%%
% start and goal are in meters 

%%%%%%%%
% start=[2.1,3.1,2.0];
% goal=[8.2,10,5.0];
%%%%%%%%%%%

start=start-map_lowerC;
goal=goal-map_lowerC;
start_node=[Height-floor(start(2)/xy_res),ceil(start(1)/xy_res),ceil(start(3)/z_res)];
start_node=double(start_node);
goal_node=[Height-floor(goal(2)/xy_res),ceil(goal(1)/xy_res),ceil(goal(3)/z_res)];
goal_node=double(goal_node);
if(start_node==goal_node)
    error('start node is same as goal node');
end
if min(start_node)<0 || min([Height,Width,Thickness]-start_node)<0 || min(goal_node)<0 || min([Height,Width,Thickness]-goal_node)<0
   error('start or goal node is out of range');
end

if map_grid(start_node(1),start_node(2),start_node(3))==1 || map_grid(goal_node(1),goal_node(2),goal_node(3))==1
    error('start or goal is part of the block');
end

%Openlist={start_node};
%OpenMAT(start_node(1),start_node(2),start_node(3))=1;
FScore(start_node(1),start_node(2),start_node(3))=0;
Open_list=[start_node(1),start_node(2),start_node(3),0];
num_expanded=1;
iffind=0;
while isempty(Open_list)==0
    
    %%%%%%% the node in openSet having the lowest fScore[] value%%%%%%%%
%     ind=find(OpenMAT);  % find no-zero in the openMat, which is the openlist
%     [i1, i2, i3] = ind2sub(size(OpenMAT), ind);
%     Open_list=[i1 i2 i3]; 
     [num_in_oplist,~]=size(Open_list);
%     for k=1:num_in_oplist
%         openI=Open_list(k,4);
%        open_FScore(k)=FScore(openI(1),openI(2),openI(3));
%     end
    %open_FScore=Open_list(:,4);
    [~,min_index]=min(Open_list); 
    %[~,min_index]=min(open_FScore); 
   % current=[Open_list(min_index,1),Open_list(min_index,2),Open_list(min_index,3)];
   if num_in_oplist==1
      % min_index=1;
       current=[Open_list(1,1),Open_list(1,2),Open_list(1,3)];
       Open_list(1,:)=[];
   else
    current=[Open_list(min_index(4),1),Open_list(min_index(4),2),Open_list(min_index(4),3)];
    Open_list(min_index(4),:)=[];
   end
    %clear   open_FScore
    
  
  % OpenMAT(current(1),current(2),current(3))=0;% openSet.Remove(current)
   ClosedMAT(current(1),current(2),current(3))=1;%closedSet.Add(current)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Literation%%%%%%%%%%
       
       for i=-1:1
           for j=-1:1
               for k=-1:1
                   if i==0 && j==0 && k==0
                       continue;
                   end
                   if IsValid(current,Height,Width,Thickness,i,j,k,ClosedMAT)
                       continue;
                   else
                   node=[current(1)+i,current(2)+j,current(3)+k];
                    num_expanded=num_expanded+1;
                   %if IsValid(node,Height,Width,Thickness,ClosedMAT)
                       if node==goal_node
                              Parent(node(1),node(2),node(3))={[current(1),current(2),current(3)]};
                             Shortest_path=generate_path(goal_node,start_node,Parent);
                             %h = msgbox('Goal node find');
                             iffind=1;
                             return;
                   elseif ~ClosedMAT(node(1),node(2),node(3))
                           GNew= GScore(current(1),current(2),current(3))+sqrt(i*i+j*j+k*k);
                           HNew=calculateH(goal_node,node);
                           FNew=GNew+2*HNew;
                           
                           if FScore(node(1),node(2),node(3))==inf || FScore(node(1),node(2),node(3))>FNew
                              %  OpenMAT(node(1),node(2),node(3))=1;
                                Open_list=[Open_list;node(1),node(2),node(3),FNew];
                                FScore(node(1),node(2),node(3))=FNew;
                                GScore(node(1),node(2),node(3))=GNew;
                                %HScore(node(1),node(2),node(3))=HNew;
                                Parent(node(1),node(2),node(3))={[current(1),current(2),current(3)]};
                           end
                        
                       end
                   end
                       
               end
           end
        
       end
        
end

if iffind==0
    Shortest_path=zeros(1,3);
else
    Shortest_path=flipud(Shortest_path);
end





%% END YOUR CODE HERE %%

end
