function [Q_trees_,D_start_new] = RRT_random(P_start, P_goal, P_obstacles)
% basic rrt, choose the nearest point in the tree 
% from both the start and goal simuteneously
% and step on

% input: points of the start, the goal and all obstacles
% output: path, or trees from start and goal
%         end of the tree from start

Q_init_ = P_start;
Q_goal_ = P_goal;

n_iteration = 5000;
step_ = 0.2;

iteration = 1;
iteration_m = 1;

xy_range = 10;
Q_init_ = [Q_init_];

n_obs = size(P_obstacles,1);

while (iteration<=n_iteration)
    flag_ob = 1;
    Q_rand_ = rand(1,2)*xy_range;
    for j = 1:size(Q_init_,1)
        dis_1_(j) = sqrt(sum((Q_rand_(1,:)-Q_init_(j,:)).^2));
        dis_2_(j) = sqrt(sum((Q_rand_(1,:)-Q_goal_(j,:)).^2));
    end
    [x,n] = min(dis_1_);
    Q_near_1_ = Q_init_(n,:);
    
    move_direction_1_ = compute_angles_(Q_near_1_,Q_rand_);
    Q_new_1_(1) = Q_near_1_(1) + step_*cos(move_direction_1_);
    Q_new_1_(2) = Q_near_1_(2) + step_*sin(move_direction_1_);
    
    [y,m] = min(dis_2_);
    Q_near_2_ = Q_goal_(m,:);
    move_direction_2_ = compute_angles_(Q_near_2_,Q_rand_);
    Q_new_2_(1) = Q_near_2_(1) + step_*cos(move_direction_2_);
    Q_new_2_(2) = Q_near_2_(2) + step_*sin(move_direction_2_);
    
    
    
    for i=1:n_obs
        dis_ob_1(j) = sqrt(sum((Q_new_1_(1,:) - P_obstacles(i,:)).^2));
        dis_ob_2(j) = sqrt(sum((Q_new_2_(1,:) - P_obstacles(i,:)).^2));
        if (dis_ob_1(j) < step_) || (dis_ob_2(j) < step_)
            flag_ob =0;
            break;
        end
    end
    
    if flag_ob
        Q_init_ = [Q_init_;Q_new_1_];
        q_newstep_1_ = [Q_near_1_;Q_new_1_];
        qtree_matix_1_(n,iteration_m+1) = 1;
        
        Q_goal_ = [Q_goal_;Q_new_2_];
        q_newstep_2_ = [Q_near_2_;Q_new_2_];
        qtree_matix_2_(m,iteration_m+1) = 1;
        
        iteration_m = iteration_m+1;
        
        iteration = iteration + 1;
        iflag = 0;
        
        for ki = 1:size(Q_init_,1)
            d_bi_1 = sqrt(sum((Q_new_2_-Q_init_(ki,:)).^2));
            if d_bi_1 <= step_
                [q_trees_init, n_start1] = find_each_path(qtree_matix_1_,ki,Q_init_);
                [q_trees_goal, n_start2] = find_each_path(qtree_matix_2_,iteration,Q_goal_);
                Q_tree_ = [q_trees_init(end:-1:1,:);q_trees_goal];
                d_start_new = n_start1;
                iflag = 1;
                break
            end
        end
        
        for kj = 1:size(Q_goal_,1)
            d_bi_2 = sqrt(sum((Q_goal_(kj,:)-Q_new_1_).^2));
            if d_bi_2 <= step_
                [q_trees_goal, n_start3] = find_each_path(qtree_matix_2_,kj,Q_goal_);
                [q_trees_init, n_start4] = find_each_path(qtree_matix_1_,iteration,Q_init_);
                Q_tree_ = [q_trees_init(end:-1:1,:);q_trees_goal];
                d_start_new = n_start4;
                iflag = 1;
                break
            end
        end
        
        if iflag
            break
        end
        
    end
    
end
Q_trees_ = [P_start;Q_tree_;P_goal];
D_start_new = d_start_new+1;
plot_a_tree(Q_trees_)

%%%%%%%
function plot_a_tree(tree)
for i =1:(size(tree,1)-1)
    qtree_ = [tree(i,:);tree(i+1,:)];
    %plot(qtree_(:,1),qtree_(:,2),'r-');
end

%%%%%%%%
function [q_trees_, n_start] = find_each_path(qtree_matrics,n,qtree)
%ni=size(qtree_matrics,2);
ni=n;
nj=1;
while ni>1
    mi= find(qtree_matrics(:,ni)==1);
    qtree_ = [qtree(mi,:);qtree(ni,:)];
    %plot(qtree_(:,1),qtree_(:,2),'r-');
    q_tree_(nj,:) = qtree(ni,:);
    ni = mi;
    nj = nj+1;
end
n_start = nj-1;
q_trees_ = q_tree_;





