%author: Zejiang Zeng
%date: 09/09/2017
function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  an obstacle.

% CMSC 828T Proj 1 Phase 1

% Output structure: 
    % Map is a cell array containing the following:
    %   --> map{1} contains a 3D logical occupancy grid
    %   --> map{2} and map{3} store the xy_res and z_res respectively
    %   --> map{4} coordinate of lower left corner
    %   --> map{5} shifted map
    %   --> map{6} number of block

% Open file, read data, close file. Comments in file marked by #
fileID = fopen(filename);
fileDat = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f','CommentStyle','#');
fclose(fileID);

%% START YOUR CODE HERE %%
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
                if j>0 && i>0 && k>0
                map_grid(j,i,k)=1; 
                end
            end
        end
    end          
end

map=cell(6,1);
map(1)={map_grid};
map(2)={xy_res};
map(3)={z_res};
map(4)={map_lowerC};
map(5)={shifted_map};
map(6)={BlockSize};
%% END YOUR CODE HERE %%

end
