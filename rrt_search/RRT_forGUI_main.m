function [ b ] = RRT_forGUI_main(  )
% test of basic codes
cla

disp('run naive rrt ...')
RRT_forGUI_naive()
pause(1)
cla
disp('run rrt by voronoi ...')
RRT_forGUI_voronoi
pause(1)
cla
disp('run rrt random ...')
RRT_forGUI_random()
pause(1)
cla
disp('run rrt connected ...')
RRT_forGUI_random_connect()
pause(1)
cla
disp('run rrt merged ...')
RRT_forGUI_random_merge()
pause(1)
cla
disp('run rrt with circle obstacles ...')
RRT_forGUI_random_with_ob(4)
pause(1)
cla
disp('run rrt with cube obstacles ...')
RRT_forGUI_random_with_ob_cube(4)