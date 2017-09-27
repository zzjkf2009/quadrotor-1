
function [H]=calculateH(goal_node,node)
 
H=sqrt((goal_node(1)-node(1))^2+(goal_node(2)-node(2))^2+(goal_node(3)-node(3))^2);
end