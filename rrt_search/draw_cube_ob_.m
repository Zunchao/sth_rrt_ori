function draw_cube_ob_(start_point,width_height)
% draw a cube obstacle, with inputs of
% init point and width and height
for i = 1:size(start_point,1)
    cubex = [start_point(i,1),start_point(i,1),start_point(i,1)+width_height(i,1),start_point(i,1)+width_height(i,1);
        start_point(i,1)+width_height(i,1),start_point(i,1),start_point(i,1)+width_height(i,1),start_point(i,1)];
    cubey = [start_point(i,2),start_point(i,2),start_point(i,2)+width_height(i,2),start_point(i,2)+width_height(i,2);
        start_point(i,2),start_point(i,2)+width_height(i,2),start_point(i,2),start_point(i,2)+width_height(i,2)];
    plot(cubex,cubey,'k-')
    hold on
end