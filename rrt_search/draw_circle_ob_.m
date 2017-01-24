function draw_circle_ob_(circle_center,r)
theta = 0:pi/100:2*pi;
for i = 1:size(circle_center,1)
    x = circle_center(i,1)+r*cos(theta);
    y = circle_center(i,2)+r*sin(theta);
    plot(circle_center(i,1),circle_center(i,2),'k*',x,y,'k-')
    hold on
end