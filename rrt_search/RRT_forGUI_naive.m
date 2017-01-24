function RRT_forGUI_naive()
% naive search , randomly choose a point in the tree, circled chaos
%
Q_init_ = [5,5];
n_iteration = 1000;
xy_range = 10;
step_ = 0.2;
iteration = 1;

while (iteration<=n_iteration)
    Q_rand_ = rand(1,2)*xy_range
    xn=size(Q_init_,1)
    Q_near_ = Q_init_(unidrnd(xn),:);
    move_direction_ = atan2(Q_rand_(2)-Q_near_(2),Q_rand_(1)-Q_near_(1));
    Q_new_(1) = Q_near_(1) + step_*cos(move_direction_);
    Q_new_(2) = Q_near_(2) + step_*sin(move_direction_);
    Q_init_ = [Q_init_;Q_new_];
    q_newstep_ = [Q_near_;Q_new_];
    plot(q_newstep_(:,1),q_newstep_(:,2),'.-')
    hold on
    axis([0 xy_range 0 xy_range])
    drawnow
    iteration = iteration + 1;
end