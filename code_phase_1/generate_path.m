%author: Zejiang Zeng
%date: 09/09/2017
function [Shortest_path]=generate_path(goal_node,start_node,Parent)
Shortest_path=goal_node;
tem_path=cell2mat(Parent(goal_node(1),goal_node(2),goal_node(3)));
while ~isequal(tem_path,start_node)
Shortest_path=[Shortest_path;tem_path];
tem_path=cell2mat(Parent(tem_path(1),tem_path(2),tem_path(3)));
end
Shortest_path=[Shortest_path;start_node];