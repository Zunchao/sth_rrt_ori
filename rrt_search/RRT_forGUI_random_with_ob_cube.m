function RRT_forGUI_random_with_ob_cube(n_ob_)
% basic rrt, choose the nearest pont in the tree, and step on
% search the space
% obstacles are all the shape of cube

n_iteration = 2000;
xy_range = 10;
Q_goal_ = [xy_range,xy_range];
step_ = 0.1;
iteration = 1;

obstacle_cube_ = rand(n_ob_,2)*xy_range;
widthHeight_ = randi([step_*100,100],n_ob_,2)/100*xy_range/3;

Q_init_ = [0,0];

draw_cube_ob_(obstacle_cube_,widthHeight_);

while (iteration < n_iteration)
    Q_rand_ = rand(1,2)*xy_range;
    for j = 1:size(Q_init_,1)
        dis_1_(j) = sqrt(sum((Q_rand_(1,:)-Q_init_(j,:)).^2));
    end
    [x,n] = min(dis_1_);
    Q_near_ = Q_init_(n,:);
    
    move_direction_ = compute_angle_(Q_near_,Q_rand_);
    Q_new_(1) = Q_near_(1) + step_*cos(move_direction_);
    Q_new_(2) = Q_near_(2) + step_*sin(move_direction_);
    
    i_flag_ = 0;
    for i = 1:n_ob_
        endpoints = [Q_near_; Q_new_];
        bool_xy = line_cube_intersect(endpoints, obstacle_cube_(i,:), widthHeight_(i,:));
        if bool_xy
            i_flag_ = i_flag_ + 1;
        end
        %{
        h_flag_ = (Q_new_(1) >= obstacle_cube_(i,1)) && ...
            (Q_new_(1) <= obstacle_cube_(i,1)+widthHeight_(i,1)) && ...
            (Q_new_(2) >= obstacle_cube_(i,2)) && ...
            (Q_new_(2) <= obstacle_cube_(i,2)+widthHeight_(i,2));
        if h_flag_ == 1
            i_flag_ = i_flag_ +1;
        end
        %}
    end
    
    if (~i_flag_)        
        Q_init_ = [Q_init_;Q_new_];
        q_newstep_ = [Q_near_;Q_new_];
        plot(Q_init_(1,1),Q_init_(1,2),'ro',q_newstep_(:,1),q_newstep_(:,2),'-')
        hold on
        axis([0 xy_range 0 xy_range])
        drawnow
        %pause(0.01)
    end
    iteration = iteration + 1;
end
iteration
size(Q_init_)