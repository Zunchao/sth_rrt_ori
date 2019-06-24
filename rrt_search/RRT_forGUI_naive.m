function RRT_forGUI_naive(p_start, p_goal)
% naive search , randomly choose a point in the tree, circled chaos
%
Q_init_ = (p_start+p_goal)/2;
n_iteration = 500;
xy_range = (sum(p_goal-p_start))/2;
step_ = 0.2;
iteration = 1;

writerObj=VideoWriter('RRT_forGUI_naive.avi');  %// 定义一个视频文件用来存动画
open(writerObj);                    %// 打开该视频文件

plot(Q_init_(1,1),Q_init_(1,2),'ro')

while (iteration<=n_iteration)
    Q_rand_ = rand(1,2)*xy_range;
    xn=size(Q_init_,1);
    Q_near_ = Q_init_(unidrnd(xn),:);
    move_direction_ = atan2(Q_rand_(2)-Q_near_(2),Q_rand_(1)-Q_near_(1));
    Q_new_(1) = Q_near_(1) + step_*cos(move_direction_);
    Q_new_(2) = Q_near_(2) + step_*sin(move_direction_);
    Q_init_ = [Q_init_;Q_new_];
    q_newstep_ = [Q_near_;Q_new_];
    
    frame = getframe;            %// 把图像存入视频文件中
    writeVideo(writerObj,frame); %// 将帧写入视频
    
    plot(q_newstep_(:,1),q_newstep_(:,2),'.-')
    %hold on
    %axis([0 xy_range 0 xy_range])
    drawnow
    iteration = iteration + 1;
end

close(writerObj); %// 关闭视频文件句柄