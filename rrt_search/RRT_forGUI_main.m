function RRT_forGUI_main()
% test of basic codes

P_start = [0 0];
P_goal = [10 10];

cla
hold on
axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
disp('run naive rrt ...')
RRT_forGUI_naive(P_start, P_goal);
pause(1)

cla
hold on
axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
disp('run rrt by voronoi ...')
RRT_forGUI_voronoi(P_start, P_goal);
pause(1)

cla
hold on
axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
disp('run rrt random ...')
RRT_forGUI_random(P_start, P_goal);
pause(1)

cla
hold on
axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
disp('run rrt connected ...')
RRT_forGUI_random_connect(P_start, P_goal);
pause(1)

cla
hold on
axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
disp('run rrt merged ...')
RRT_forGUI_random_merge(P_start, P_goal);
pause(1)

cla
hold on
axis([P_start(1,1)-1, P_goal(1,1)+1, P_start(1,2)-1, P_goal(1,2)+1])
disp('run rrt with circle obstacles ...')
RRT_forGUI_random_with_ob(P_goal, 4)
pause(1)

cla
hold on
disp('run rrt with cube obstacles ...')
RRT_forGUI_random_with_ob_cube(P_start, P_goal, 4)















