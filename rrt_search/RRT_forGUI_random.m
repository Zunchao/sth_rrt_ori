function RRT_forGUI_random(p_start, p_goal)
% basic rrt, choose the nearest pont in the tree, and step on
% search the space
Q_init_ = (p_start+p_goal)/2;
n_iteration = 500;

xy_range = (sum(p_goal-p_start))/2;
Q_goal_ = [xy_range,xy_range];
step_ = 0.2;
iteration = 1;

writerObj=VideoWriter('RRT_forGUI_random.avi');  %// 定义一个视频文件用来存动画
open(writerObj);                    %// 打开该视频文件

while (iteration<=n_iteration)
    Q_rand_ = rand(1,2)*xy_range;
    for j = 1:size(Q_init_,1)
        dis_1_(j) = sqrt(sum((Q_rand_(1,:)-Q_init_(j,:)).^2));
    end
    [x,n] = min(dis_1_);
    Q_near_ = Q_init_(n,:);
    %move_direction_ = atan2(Q_rand_(2)-Q_near_(2),Q_rand_(1)-Q_near_(1));
    move_direction_ = compute_angle_(Q_near_,Q_rand_);
    Q_new_(1) = Q_near_(1) + step_*cos(move_direction_);
    Q_new_(2) = Q_near_(2) + step_*sin(move_direction_);
    
    Q_init_ = [Q_init_;Q_new_];
    q_newstep_ = [Q_near_;Q_new_];
    
    frame = getframe;            %// 把图像存入视频文件中
    writeVideo(writerObj,frame); %// 将帧写入视频
    
    plot(Q_init_(1,1),Q_init_(1,2),'ro',q_newstep_(:,1),q_newstep_(:,2),'-')
    %hold on
    %axis([0 xy_range 0 xy_range])
    drawnow
    
    iteration = iteration + 1;
end
Q_init_;
close(writerObj); %// 关闭视频文件句柄