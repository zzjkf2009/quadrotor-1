%author: Zejiang Zeng
%date: 09/09/2017
clear
tic
filename='sample_maps/map0.txt';
fileID = fopen(filename);
fileDat = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f','CommentStyle','#');
fclose(fileID);
%celldisp(data);
%%%%%%%%%%
% Paramaters
xy_res=0.1;
z_res=0.12;
margin=0.01;
%%%%%%%%%%
[BlockSize,~]=size(fileDat{2});
map_info=zeros(BlockSize,9);
if BlockSize==1
    info_num=6;
else
    info_num=9;
end
for i=1:BlockSize
    for j=1:info_num

        map_info(i,j)=fileDat{j+1}(i);

    end
end
clear i j;
map_lowerC=[map_info(1,1),map_info(1,2),map_info(1,3),];
%map_upperC=[map_info(1,4),map_info(1,5),map_info(1,6),];

temp=zeros(BlockSize,3);
for n=1:BlockSize
    temp(n,:)=map_lowerC;
end
temp_shift=[temp,temp,zeros(BlockSize,3)];
% build the shiftted map which the lower conner is (0,0,0)
shifted_map=map_info-temp_shift;
% Build the map grid by a row_num*col_num*layer_num matrix
row_num=ceil(shifted_map(1,5)/xy_res);

col_num=ceil(shifted_map(1,4)/xy_res);

% find how many layers for the map grid

layer_num=ceil(shifted_map(1,6)/z_res);

% initialize the 3d grid map by a (row_num,col_num,layer_num) matrix
map_grid=zeros(row_num,col_num,layer_num);
%%%%
%FIND THE LOCATION OF OBSTICALES
for each_block=2:BlockSize
    block_x_init=floor(shifted_map(each_block,1)/xy_res)+1;
 
    block_x_end=ceil(shifted_map(each_block,4)/xy_res);

    block_y_init=row_num-floor(shifted_map(each_block,2)/xy_res);

    block_y_end=row_num-ceil(shifted_map(each_block,5)/xy_res)+1;

%     block_z_init=floor(shifted_map(each_block,3)/z_res)+1;
%     block_z_end=ceil(shifted_map(each_block,6)/z_res);
   if rem(shifted_map(each_block,3),z_res)+margin<z_res
       block_z_init=floor(shifted_map(each_block,3)/z_res)+1;
   else
        block_z_init=ceil(shifted_map(each_block,3)/z_res)+1;
   end
   
   if rem(shifted_map(each_block,6),z_res)>margin
       block_z_end=ceil(shifted_map(each_block,6)/z_res);
   else
       block_z_end=floor(shifted_map(each_block,6)/z_res);
   end
   
    for i=block_x_init:block_x_end
        for j=block_y_end:block_y_init
            for k=block_z_init:block_z_end
                
                map_grid(j,i,k)=1;
                 
            end
        end
    end          
end
 clear i j k;
%%%%%% Finish mapping and start finding path%%%%
% where map is a M-N-K matrix 
[Height,Width,Thickness]=size(map_grid);
GScore=zeros(Height,Width,Thickness);   %Matrix keeping track of G-scores 
FScore=single(inf(Height,Width,Thickness));  %Matrix keeping track of F-scores (only open list) 
HScore=single(zeros(Height,Width,Thickness));   %Heuristic matrix
OpenMAT=false(Height,Width,Thickness); %Matrix keeping of open grid cells
ClosedMAT=false(Height,Width,Thickness);   %Matrix keeping track of closed grid cells
ClosedMAT(map_grid==1)=1;  %Adding object-cells to closed matrix
Parent=cell(Height,Width,Thickness); %store parent node into cells
%%%%%%%%%%%%%%
% start and goal are in meters 

%%%%%%%%
start=[2.1,3.1,2.0];
goal=[8.2,10,5.0];
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
OpenMAT(start_node(1),start_node(2),start_node(3))=1;
FScore(start_node(1),start_node(2),start_node(3))=0;
Open_list=[start_node(1),start_node(2),start_node(3),0];
litration=1;
num_in_oplist=1;
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
       min_index=1;
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
                   %if IsValid(node,Height,Width,Thickness,ClosedMAT)
                       if node==goal_node
                              Parent(node(1),node(2),node(3))={[current(1),current(2),current(3)]};
                             Shortest_path=generate_path(goal_node,start_node,Parent);
                             msgbox('Goal node find');
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
        litration=litration+1;
end
   
toc

%%
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
fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
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
fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
patch('Vertices',vert,'Faces',fac,...
      'EdgeColor','none','FaceColor','yellow','LineWidth',2);
view(3)
axis equal
hold on   
    
end    

