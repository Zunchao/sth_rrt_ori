%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算斥力分程序：
%斥力计算
function [Yrerxx,Yreryy,Yataxx,Yatayy]=compute_repulsion_(Pstart,Pgoal,Pobstacle,angle_at,angle_re,Po,a)
%输入参数为当前坐标，Xsum是目标和障碍的坐标向量，增益常数,障碍，目标方向的角度
Rat = sum((Pstart-Pgoal).^2);%路径点和目标的距离平方
rat = sqrt(Rat);%路径点和目标的距离
n = size(Pobstacle,1);

for i=1:n
    Rrei(i) = sum((Pstart(1,:)-Pobstacle(i,:)).^2);%路径点和障碍的距离平方
    rre(i) = sqrt(Rrei(i));%路径点和障碍的距离保存在数组rrei中
    d_gos = sum((Pgoal(1,:)-Pobstacle(i,:)).^2);
    d_go = sqrt(d_gos);
    K_rep = randi([1 3]);%计算斥力的增益系数
    if (rre(i)>Po)||(rat<rre(i))%如果每个障碍和路径的距离大于障碍影响距离，斥力令为0
        Yrerx(i)=0;
        Yrery(i)=0;
        Yatax(i)=0;
        Yatay(i)=0;
    else
        if rre(i)<Po/2
            Yrer(i)=K_rep*(1/rre(i)-1/Po)*(1/Rrei(i))*(rat^a);%分解的Fre1向量
            Yata(i)=0.5*a*K_rep*((1/rre(i)-1/Po)^2)*(rat^(1-a));%分解的Fre2向量
            Yrerx(i)=Yrer(i)*cos(angle_re(i));%angle_re(i)=Y(i+1)
            Yrery(i)=Yrer(i)*sin(angle_re(i));
            Yatax(i)=Yata(i)*cos(angle_at);%angle_at=Y(1)
            Yatay(i)=Yata(i)*sin(angle_at);
        else
            Yrer(i)=K_rep*(1/rre(i)-1/Po)*1/Rrei(i)*Rat;%分解的Fre1向量
            Yata(i)=0.5*a*K_rep*((1/rre(i)-1/Po)^2)*rat;%分解的Fre2向量
            Yrerx(i)=Yrer(i)*cos(angle_re(i));%angle_re(i)=Y(i+1)
            Yrery(i)=Yrer(i)*sin(angle_re(i));
            Yatax(i)=Yata(i)*cos(angle_at);%angle_at=Y(1)
            Yatay(i)=Yata(i)*sin(angle_at);
        end
    end%判断距离是否在障碍影响范围内
end
Yrerxx=-sum(Yrerx);%叠加斥力的分量
Yreryy=-sum(Yrery);
Yataxx=-sum(Yatax);
Yatayy=-sum(Yatay);
