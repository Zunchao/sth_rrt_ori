%计算引力分程序：
function [Yatx,Yaty]=compute_attract_(Pstart,Pgoal,angle)
%输入参数为当前坐标，目标坐标，增益常数,分量和力的角度
%把路径上的临时点作为每个时刻的Xgoal
R=sum((Pstart-Pgoal).^2);%路径点和目标的距离平方
r=sqrt(R);%路径点和目标的距离
k_attr=randi([15 25]);%计算引力需要的增益系数
Yatx=k_attr*r*cos(angle);%angle=Y(1)
Yaty=k_attr*r*sin(angle);