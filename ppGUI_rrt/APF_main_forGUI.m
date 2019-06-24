function [flag_apf, step_in_all, Pgoals_path, P_obstaclespath] = APF_main_forGUI(P_start, P_goal, P_obstacles, Angle_goal_, num_mov_ob)

%k_attr=25;%计算引力需要的增益系数
%初始化
%m=5;%计算斥力的增益系数，都是自己设定的。%在斥力函数中改为随机
Po=2;%障碍影响距离，当障碍和车的距离大于这个距离时，斥力为0，即不受该障碍的影响。也是自己设定。
a=0.5;
%lstep=0.2;%步长
Jnum=1000;%循环迭代次数

P_path = P_start;%j=1循环初始，将车的起始坐标赋给Xj

obstacle_num = size(P_obstacles,1);

%num = floor(obstacle_num/2);

ob_vel=0.2;
P_obstacles(1:num_mov_ob,:);

if num_mov_ob
    Angle_moving_ob = compute_angles_(P_start*0.35+P_goal*0.65,P_obstacles(1:num_mov_ob,:));
else
    Angle_moving_ob = 0;
end

writerObj=VideoWriter('pp_apf_o.avi');  %// 定义一个视频文件用来存动画
open(writerObj);                    %// 打开该视频文件

%Angle_goal_ = randi([-180,180])*pi/180;
if isempty(Angle_goal_)
    flag_Movinggoal = 0;
else
    flag_Movinggoal = 1;
end
i2=1;
k=1;
flag_apf = 0;
%***************初始化结束，开始主体循环******************
for j=1:Jnum%循环开始
    
    frame = getframe;            %// 把图像存入视频文件中
    writeVideo(writerObj,frame); %// 将帧写入视频
    
    Goal(j,:)=P_path(:);%Goal是保存车走过的每个点的坐标。刚开始先将起点放进该向量。
    %调用计算角度模块
    for i1=1:num_mov_ob
        %P_obstacles(num+i1-1,1)=P_obstacles(num+i1-1,1)+ob_vel*cos(Angle_moving_ob(i1)+pi);
        %P_obstacles(num+i1-1,2)=P_obstacles(num+i1-1,2)+ob_vel*sin(Angle_moving_ob(i1)+pi);
        pob=P_obstacles;
        pob(i1,:) = [];
        pointall = [pob; P_goal];
        P_ob = compute_new_without_avoidance(P_obstacles(i1,:), Angle_moving_ob(i1)+pi, pointall,Po,a, ob_vel);
        P_obstacles(i1,:) = P_ob;
        P_obstaclespath(i1,:,i2) = P_ob;
    end
    i2=i2+1;
    Angle_obstaclePath = compute_angles_(P_path,P_obstacles);%Theta是计算出来的车和障碍，
    %和目标之间的与X轴之间的夹角，统一规定角度为逆时针方向，用这个模块可以计算出来。
    %调用计算引力模块
    Angle_goalPath = compute_angles_(P_path,P_goal);%Theta（1）是车和目标之间的角度，目标对车是引力。
    %为了后续计算斥力在引力方向的分量赋值给angle_at
    [Fatx,Faty] = compute_attract_(P_path,P_goal,Angle_goalPath);
    %计算出目标对车的引力在x,y方向的两个分量值。
    %调用计算斥力模块
    [Frerxx,Freryy,Fataxx,Fatayy]=compute_repulsion_...
        (P_path,P_goal,P_obstacles,Angle_goalPath,Angle_obstaclePath,Po,a);
    % 计算出斥力在x,y方向的分量数组。
    %计算合力和方向，这有问题，应该是数，每个j循环的时候合力的大小应该是一个唯一的数，
    %不是数组。应该把斥力的所有分量相加，引力所有分量相加。
    Fsumxj=Fatx+Frerxx+Fataxx;%x方向的合力
    Fsumyj=Faty+Freryy+Fatayy;%y方向的合力
    %Position_angle(j)=atan2(Fsumyj,Fsumxj);%合力与x轴方向的夹角向量%计算车的下一步位置
    Position_angle_force = compute_angles_([0 0],[Fsumxj,Fsumyj]);
    Position_angle_gp = compute_angles_(P_goal, P_path);
    if (Position_angle_force < Position_angle_gp + 45/180*pi) ...
            && (Position_angle_force > Position_angle_gp - 45/180*pi)
        Fsumxj=(Fatx+Frerxx+Fataxx)/3;%x方向的合力
        Fsumyj=(Faty+Freryy+Fatayy)/3;%y方向的合力
        disp('local minimal ?')
        %set(handles.edit_achtung,'String','local minimal?')
    end
    Position_angle_force = compute_angles_([0 0],[Fsumxj,Fsumyj]);
    lstep = randi([10 20])/100;
    P_next(1)=P_path(1)+lstep*cos(Position_angle_force);
    P_next(2)=P_path(2)+lstep*sin(Position_angle_force);%保存车的每一个位置在向量中
    P_path=P_next;
    %判断
    d_path_goal = sqrt(sum((P_path-P_goal).^2));
    if(d_path_goal < 0.05)
        %是应该完全相等的时候算作到达，
        %还是只是接近就可以？现在按完全相等的时候编程。
        %记录迭代到多少次，到达目标。
        flag_apf = 1;
        %set(handles.edit_display,'String','plan succeed!')
        break;
        %记录此时的j值
    end%如果不符合if的条件，重新返回循环，继续执行。
    if (flag_Movinggoal)
        
        pointall = [P_obstacles];
        P_goals = compute_new_without_avoidance(P_goal, Angle_goal_, pointall,Po,a, ob_vel/4);
        P_goal = P_goals ;
        %{
        
        P_pathgoal = P_goal;
        P_goals(1) = P_goal(1) + 3*cos(Angle_goal_);
        P_goals(2) = P_goal(2) + 3*sin(Angle_goal_);
        Angle_obstaclePathgoal = compute_angle_(P_pathgoal,P_obstacles);
        [pFatx,pFaty] = compute_attract_(P_pathgoal,P_goals,Angle_goal_);
        [pFrerxx,pFreryy,pFataxx,pFatayy]=compute_repulsion_(P_pathgoal,P_goals,P_obstacles,Angle_goal_,Angle_obstaclePathgoal,Po,a);
        % 计算出斥力在x,y方向的分量数组。
        %计算合力和方向，这有问题，应该是数，每个j循环的时候合力的大小应该是一个唯一的数，
        %不是数组。应该把斥力的所有分量相加，引力所有分量相加。
        pFsumxj=pFatx+pFrerxx+pFataxx;%x方向的合力
        pFsumyj=pFaty+pFreryy+pFatayy;%y方向的合力
        Position_angle_forcegoal = compute_angle_([0 0],[pFsumxj,pFsumyj]);
        Angle_goal_=Position_angle_forcegoal;
        P_goal(1) = P_goal(1) + ob_vel/4*cos(Angle_goal_);
        P_goal(2) = P_goal(2) + ob_vel/4*sin(Angle_goal_);
        %}
        Pgoals(k,:) = P_goal;
        k=k+1;
    else
        Pgoals=P_goal;
        
    end
    
    step_in_all=j;
    if step_in_all == Jnum
        %set(handles.edit_display,'String','plan end!')
    end
    for i3=1:num_mov_ob
        plot(P_obstacles(i3,1),P_obstacles(i3,2),'bo');
    end
    plot(P_goal(1),P_goal(2),'k.');
    plot(Goal(j,1),Goal(j,2),'g<');
    %set(handles.edit_display,'String',num2str(step_in_all))
    drawnow;
    pause(0.1)
end%大循环结束

Goal(step_in_all+1,:)=P_goal(:);
%Goal(step_in_all,:) = P_goal(:);%把路径向量的最后一个点赋值为目标
%*****************画出障碍，起点，目标，路径点*************************
%画出路径
size(Goal);
%路径向量Goal是二维数组,X,Y分别是数组的x,y元素的集合，是两个一维数组。
%plot(Goal(:,1),Goal(:,2),'g.');
plot(P_goal(:,1),P_goal(:,2),'g.');
%str = strcat('the step number is :', num2str(step_in_all));
%set(handles.edit_display,'String',num2str(step_in_all))
Pgoals_path = Pgoals;
%P_obstaclespath;
close(writerObj); %// 关闭视频文件句柄

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算角度分程序：
%function Angle = compute_angle_(Ppath,Pgoa_obs)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算引力分程序：
function [Yatx,Yaty]=compute_attract_(Pstart,Pgoal,angle)
%输入参数为当前坐标，目标坐标，增益常数,分量和力的角度
%把路径上的临时点作为每个时刻的Xgoal
R=sum((Pstart-Pgoal).^2);%路径点和目标的距离平方
r=sqrt(R);%路径点和目标的距离
k_attr=randi([15 25]);%计算引力需要的增益系数
Yatx=k_attr*r*cos(angle);%angle=Y(1)
Yaty=k_attr*r*sin(angle);

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
%}