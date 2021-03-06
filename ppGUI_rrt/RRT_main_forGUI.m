function RRT_main_forGUI(pstart, pgoal, pobstacles, Angle_goal_, num_mov_ob)
% path planning when obstacles or goal move

% input: points of start, the goal and all obstacles,
%        the moving direction of the goal,
%        and the number of moving obsatcles

obstacle_num = size(pobstacles,1);
Po=2;
a=0.5;
ob_vel=0.2;
k=1;
%num = floor(obstacle_num/2);

if num_mov_ob
    Angle_moving_ob = compute_angles_(pstart*0.35+pgoal*0.65,pobstacles(1:num_mov_ob,:));
else
    Angle_moving_ob = 0;
end

step_=0.2;

pstartnext = pstart;
dis_end = sqrt(sum((pstart-pgoal).^2));
ob_vel=0.2;
dis_path_ob = 1000;

[Q_path_,dir_start_new] =  RRT_random(pstart, pgoal, pobstacles);

writerObj=VideoWriter('pp_rrt_o.avi');  %// 定义一个视频文件用来存动画
open(writerObj);                    %// 打开该视频文件

while dis_end > step_
    
    for i = 1:dir_start_new-1
        
        flag_avoid=[];
        frame = getframe;            %// 把图像存入视频文件中
        writeVideo(writerObj,frame); %// 将帧写入视频
        if num_mov_ob
            for i1=1:num_mov_ob
                %pobstacles(num+i1-1,1)=pobstacles(num+i1-1,1)+ob_vel*cos(Angle_moving_ob(i1)+pi);
                %pobstacles(num+i1-1,2)=pobstacles(num+i1-1,2)+ob_vel*sin(Angle_moving_ob(i1)+pi);
                pob=pobstacles;
                pob(i1,:) = [];
                pointall = [pob; pgoal];
                P_ob = compute_new_without_avoidance(pobstacles(i1,:), Angle_moving_ob(i1)+pi, pointall,Po,a, ob_vel);
                pobstacles(i1,:) = P_ob;
                dis_path_ob = sqrt(sum((Q_path_(i+1,:)-pobstacles(i1,:)).^2));
                plot(pobstacles(i1,1),pobstacles(i1,2),'c*')
                
                if (dis_path_ob < step_)
                    flag_avoid(i1) = 1;
                    disp('WARNING: obstacle near!')
                    dis_path_ob = step_+1;
                else
                    flag_avoid(i1) = 0;
                    dis_path_ob = step_+1;
                end
            end
            
        else
            flag_avoid = 0;
        end
        
        %flag_avoid
        if (isempty(Angle_goal_)==0)
            P_goals = compute_new_without_avoidance(pgoal, Angle_goal_, pobstacles,Po,a, ob_vel/5);
            pgoal = P_goals ;
            plot(pgoal(1),pgoal(2),'mo');
        end
        
        if (sum(flag_avoid)==0)
            ppath=[Q_path_(i,:);Q_path_(i+1,:)];
            plot(ppath(:,1),ppath(:,2),'g.')
            %drawnow
            pause(0.1)
        else
            break;
        end
    end
    
    pstart = Q_path_(i+1,:);
    Q_path_(end,:);
    Q_path_ = [];
    [Q_path_,dir_start_new] = RRT_random(pstart, pgoal, pobstacles);
    dis_end = sqrt(sum((pstart-pgoal).^2));
    
end

close(writerObj); %// 关闭视频文件句柄
%{
if (isempty(Angle_goal_)==0)
    pgoal(1) = pgoal(1) + ob_vel/4*cos(Angle_goal_);
    pgoal(2) = pgoal(2) + ob_vel/4*sin(Angle_goal_);
    plot(pgoal(1),pgoal(2),'ko')
end
%
dir_start_new =  RRT_random(pstart, pgoal, pobstacles);
pstart_new(1) = pstart(1) + step_main*cos(dir_start_new);
pstart_new(2) = pstart(2) + step_main*sin(dir_start_new);
pstartm=[pstart;pstart_new];
plot(pstartm(:,1),pstartm(:,2),'k-')
drawnow

pstart = pstart_new;
dis_end = sqrt(sum((pstart-pgoal).^2));
end
%}
end

