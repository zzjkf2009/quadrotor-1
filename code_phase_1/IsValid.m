%author: Zejiang Zeng
%date: 09/09/2017
% function [ifvalid]=IsValid(node,Height,Width,Thickness,ClosedMAT)
%  if  min(node)<=0 || min([Height,Width,Thickness]-node)<0 ||ClosedMAT(node(1),node(2),node(3))==1
%          ifvalid=0;
%  else 
%      ifvalid=1;
%  end
% end

function [ifvalid]=IsValid(current,Height,Width,Thickness,i,j,k,ClosedMAT)
 if current(1)+i<=0 || Height-(current(1)+i)<0||current(2)+j<=0 || Width-(current(2)+j)<0||current(3)+k<=0 || Thickness-(current(3)+k)<0 ||ClosedMAT(current(1)+i,current(2)+j,current(3)+k)==1
         ifvalid=1;
 else 
     ifvalid=0;
 end
end