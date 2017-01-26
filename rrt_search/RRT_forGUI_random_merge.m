function RRT_forGUI_random_merge()
% basic rrt, choose the nearest point in the tree, and step on
% from two starts, two trees until points connected
% search the space
Q_init_ = [0,0];
Q_goal_ = [10,9];
cla
n_iteration = 1000;
xy_range = 10;
%Q_goal_ = [xy_range,xy_range];
step_ = 0.1;
iteration = 1;
iteration_m = 1;

while (iteration<=n_iteration)
    Q_rand_ = rand(1,2)*xy_range;
    for j = 1:size(Q_init_,1)
        dis_1_(j) = sqrt(sum((Q_rand_(1,:)-Q_init_(j,:)).^2));
        dis_2_(j) = sqrt(sum((Q_rand_(1,:)-Q_goal_(j,:)).^2));
    end
    [x,n] = min(dis_1_);
    Q_near_1_ = Q_init_(n,:);
    %move_direction_ = atan2(Q_rand_(2)-Q_near_(2),Q_rand_(1)-Q_near_(1));
    move_direction_1_ = compute_angle_(Q_near_1_,Q_rand_);
    Q_new_1_(1) = Q_near_1_(1) + step_*cos(move_direction_1_);
    Q_new_1_(2) = Q_near_1_(2) + step_*sin(move_direction_1_);
    
    [y,m] = min(dis_2_);
    Q_near_2_ = Q_goal_(m,:);
    move_direction_2_ = compute_angle_(Q_near_2_,Q_rand_);
    Q_new_2_(1) = Q_near_2_(1) + step_*cos(move_direction_2_);
    Q_new_2_(2) = Q_near_2_(2) + step_*sin(move_direction_2_);
    
    Q_init_ = [Q_init_;Q_new_1_];
    q_newstep_1_ = [Q_near_1_;Q_new_1_];
    qtree_matix_1_(n,iteration_m+1) = 1;
    
    Q_goal_ = [Q_goal_;Q_new_2_];
    q_newstep_2_ = [Q_near_2_;Q_new_2_];
    qtree_matix_2_(m,iteration_m+1) = 1;
    
    iteration_m = iteration_m+1;
    
    plot(Q_init_(1,1),Q_init_(1,2),'ro',Q_goal_(1,1),Q_goal_(1,2),'ro');
    hold on
    plot(q_newstep_1_(:,1),q_newstep_1_(:,2),'+',q_newstep_2_(:,1),q_newstep_2_(:,2),'o')
    
    axis([0 xy_range 0 xy_range])
    drawnow
    
    iteration = iteration + 1;
    iflag = 0;
    
    for ki = 1:size(Q_init_,1)
        d_bi_ = sqrt(sum((Q_new_2_-Q_init_(ki,:)).^2));
        if d_bi_ <= step_
            [q_trees_, n_start] = find_each_path(qtree_matix_1_,ki,Q_init_);
            [q_trees_, n_start] = find_each_path(qtree_matix_2_,iteration,Q_goal_);
            iflag = 1;
            break
        end
    end
    
    for kj = 1:size(Q_goal_,1)
        d_bi_ = sqrt(sum((Q_goal_(kj,:)-Q_new_1_).^2));
        if d_bi_ <= step_
            [q_trees_, n_start] = find_each_path(qtree_matix_2_,kj,Q_goal_);
            [q_trees_, n_start] = find_each_path(qtree_matix_1_,iteration,Q_init_);
            iflag = 1;
            break
        end
    end
    
    if iflag
        break
    end
end
Q_init_;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [q_trees_, n_start] = find_each_path(qtree_matrics,n,qtree)
ni=n;
nj=1;
while ni>1    
    mi= find(qtree_matrics(:,ni)==1);
    qtree_ = [qtree(mi,:);qtree(ni,:)];    
    plot(qtree_(:,1),qtree_(:,2),'r-');
    q_tree_(nj,:) = qtree(ni,:);
    ni=mi;
    nj = nj+1;
end
n_start = ni;
q_trees_ = q_tree_;