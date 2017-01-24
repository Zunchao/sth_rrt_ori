function bool_intersect = line_cube_intersect( endpoints, start_point, width_height )
% check whether this line intersects with a cube
% input endpoints of the line , startponits and widthheights of the cube
% output bool

%bulid line
%y=ax+b;
%x>min(endpoints(:,1))
%x<max(endpoints(:,1))
%y>min(endpoints(:,2))
%y<max(endpoints(:,2))
deltay = endpoints(1,2)-endpoints(2,2);
deltax = endpoints(1,1)-endpoints(2,1);
if ~deltax
    a = 0;
else
    a = deltay/deltax;
end
b = endpoints(1,2) - a*endpoints(1,1);
x_min = min(endpoints(:,1));
x_max = max(endpoints(:,1));
y_min = min(endpoints(:,2));
y_max = max(endpoints(:,2));
% 4 lines of the cube
% x = start_point(1)
% x = start_point(1) + width
% y = start_point(2)
% y = start_point(2) + height
x1 = start_point(1,1);
x2 = start_point(1,1) + width_height(1,1);
y1 = start_point(1,2);
y2 = start_point(1,2) + width_height(1,2);

if ~a
    bool_b = (b >= y1) && (b <= y2) ;
    % intersect with y=x1 or y=x2 ?
    bool_xy = (bool_b && (x1 <= x_max) && (x1 >= x_min)) || (bool_b && (x2 <= x_max) && (x2 >= x_min));
else
    % intersect with y=x1 ?
    y11 = a*x1+b;
    bool_y11 = (y11 >= y_min) && (y11 <= y_max) && (x1 <= x_max) && (x1 >= x_min) ...
        && (y11 >= y1) && (y11 <= y2);
    % intersect with y=x2 ?
    y21 = a*x2+b;
    bool_y21 = (y21 >= y_min) && (y21 <= y_max) && (x2 <= x_max) && (x2 >= x_min) ...
        && (y21 >= y1) && (y21 <= y2);
    % intersect with y=y1 ?
    x11 = (y1 -b)/a;
    bool_x11 = (y1 >= y_min) && (y1 <= y_max) && (x11 <= x_max) && (x11 >= x_min) ...
        && (x11 >= x1) && (x11 <= x2);
    % intersect with y=y2 ?
    x21 = (y2 -b)/a;
    bool_x21 = (y2 >= y_min) && (y2 <= y_max) && (x21 <= x_max) && (x21 >= x_min) ...
        && (x21 >= x1) && (x21 <= x2);
    
    bool_xy = bool_y11 || bool_y21 || bool_x11 || bool_x21; 
end
bool_intersect = bool_xy;




