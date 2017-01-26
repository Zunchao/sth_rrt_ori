function [Q_trees_,D_start_new] = RRT_random(P_start, P_goal, P_obstacles)
% basic rrt, choose the nearest point in the tree, 

% build trees from both start and goal simuteneously
% and step on

% inputs : Points of start, goal and all the obstacles
% outputs: a path, or 2 trees from the start and the goal
%         end of the tree from start

Q_init_ = P_start;
Q_goal_ = P_goal;

n_iteration = 5000;
step_ = 0.2;

iteration = 1;
iteration_m = 1;

xy_range = 10;

n_obs = size(P_obstacles,1);

qtree_matix_1_(1,1) = 1;
qtree_matix_2_(1,1) = 1;

dis_end = sqrt(sum((P_start-P_goal).^2));

if (dis_end <= step_)
    Q_tree_ = [P_start;P_goal];
    d_start_new = 2;
end

while (dis_end > step_)%iteration<=n_iteration)
    flag_ob = 1;
    
    ax = abs(P_start(1,1)-P_goal(1,1))+1.5;
    ay = abs(P_start(1,2)-P_goal(1,2))+1.5;
    
    rax = rand(1)*ax+min([P_start(1,1),P_goal(1,1)]);
    ray = rand(1)*ay+min([P_start(1,2),P_goal(1,2)]);
    
    Q_rand_ = [rax, ray];
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
        %plot(q_newstep_1_(:,1),q_newstep_1_(:,2),'+',q_newstep_2_(:,1),q_newstep_2_(:,2),'o')
        iteration_m = iteration_m+1;
        
        iteration = iteration + 1;
        iflag = 0;
        
        for ki = 1:size(Q_init_,1)
            d_bi_1 = sqrt(sum((Q_new_2_-Q_init_(ki,:)).^2));
            if d_bi_1 <= step_
                [q_trees_init, n_start1] = find_each_path(qtree_matix_1_,ki,Q_init_);
                [q_trees_goal, n_start2] = find_each_path(qtree_matix_2_,iteration,Q_goal_);
                Q_tree_ = [q_trees_init(end:-1:1,:);q_trees_goal];
                %size(q_trees_goal);
                %size(q_trees_init);
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
                %size(q_trees_goal)
                %size(q_trees_init)
                iflag = 1;
                break
            end
        end
        
        
        if iflag
            break
        end
    end
end
Q_trees_ = [Q_tree_];
%size(Q_trees_)
D_start_new = d_start_new;
plot_a_tree(Q_trees_)
end



%%
function plot_a_tree(tree)
% plot the path
for i =1:(size(tree,1)-1)
    qtree_ = [tree(i,:);tree(i+1,:)];
    %plot(qtree_(:,1),qtree_(:,2),'b.-');
    %hold on
end
end

%%
function [q_trees_, n_start] = find_each_path(qtree_matrics,n,qtree)
% find a path in the tree
%ni=size(qtree_matrics,2);
ni=n;
nj=1;
if (ni==1)
    q_tree_ = qtree(1,:);
    nj=2;
end

while ni>1
    mi = find(qtree_matrics(:,ni)==1);
    qtree_ = [qtree(mi,:);qtree(ni,:)];
    plot(qtree_(:,1),qtree_(:,2),'r-');
    q_tree_(nj,:) = qtree(ni,:);
    ni = mi;
    nj = nj+1;
    if (ni==1)
        q_tree_(nj,:) = qtree(ni,:);
        break;
    end
end
n_start = nj;
q_trees_ = q_tree_;
end

