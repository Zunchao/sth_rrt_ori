function [ b ] = RRT_forGUI_voronoi(  )
% guide by voronoi, add a new point to the tree, and voronoi the tree
Q_init_ = [5,5];
n_iteration =2000;
xy_range = 10;
n_voronoi = 1;

step_ = 0.1;

plot(Q_init_(1,1),Q_init_(1,2),'ro')
axis([0 xy_range 0 xy_range])
hold on
iteration = 3;
Q_rand_= rand(2,2)*xy_range;
plot(Q_rand_(:,1),Q_rand_(:,2),'r+')

for i = 1:2
    move_direction_ = compute_angle_(Q_init_,Q_rand_(i,:));
    Q_new_1_(i,1) = Q_init_(1,1) + step_*cos(move_direction_);
    Q_new_1_(i,2) = Q_init_(1,2) + step_*sin(move_direction_);
    q_newstep_ = [Q_init_;Q_new_1_(i,:)];
    
    q_newstep_1_(:,i) = [Q_init_(1,1);Q_new_1_(i,1)];
    q_newstep_2_(:,i) = [Q_init_(1,2);Q_new_1_(i,2)];
    
    plot(Q_init_(1,1),Q_init_(1,2),'r<',q_newstep_(:,1),q_newstep_(:,2),'b-')
end
Q_init_=[Q_init_;Q_new_1_];

[vx,vy] = voronoi(Q_init_(:,1),Q_init_(:,2));
%plot(Q_init_(:,1),Q_init_(:,2),'b.',vx,vy,'g-')

while iteration < n_iteration
    Q_rand_= rand(1,2)*xy_range;
    %plot(Q_rand_(1,1),Q_rand_(1,2),'k+')
    
    for j = 1:size(Q_init_,1)
        dis_1_(j) = sqrt(sum((Q_rand_(1,:)-Q_init_(j,:)).^2));
    end
    [x,n] = min(dis_1_);
    Q_near_ = Q_init_(n,:);
    
    move_direction_ = compute_angle_(Q_near_,Q_rand_);
    Q_new_(1,1) = Q_near_(1,1) + step_*cos(move_direction_);
    Q_new_(1,2) = Q_near_(1,2) + step_*sin(move_direction_);
    
    q_newstep_ = [Q_near_;Q_new_];
    %plot(Q_init_(1,1),Q_init_(1,2),'ro',q_newstep_(:,1),q_newstep_(:,2),'b-')
    q_newstep_1_(:,iteration) = [Q_near_(1,1);Q_new_(1,1)];
    q_newstep_2_(:,iteration) = [Q_near_(1,2);Q_new_(1,2)];
    Q_init_=[Q_init_;Q_new_];
    [vx,vy] = voronoi(Q_init_(:,1),Q_init_(:,2));
    plot(Q_init_(1,1),Q_init_(1,2),'ro',q_newstep_1_(:,:),q_newstep_2_(:,:),'b-',vx,vy,'g-')
    hold off
    axis([0 xy_range 0 xy_range])
    drawnow
    %pause(0.01)
    iteration = iteration+1;
end
