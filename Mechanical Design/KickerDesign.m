ball_diameter = 1.68;
ball_radius = ball_diameter/2;
ball_area = pi*ball_radius^2;
syms theta;
eqn = ball_area*((2*pi - theta)/(2*pi)) +...
    ball_radius^2*cos(theta/2)*sin(theta/2) == .8*ball_area;
theta_d = solve(eqn);
allowed_into_robot = ball_radius - ball_radius*cos(theta_d/2)
height_off_the_ground_at_face_of_robot =...
    ball_radius - ball_radius*sin(theta_d/2)
height_off_the_groud_at_45 = ball_radius - ball_radius*sin(pi/4)

angles_in_hexagon = 2*pi/6*[0,1,2,3,4,5,6];
point_in_hexagon = 4*[cos(angles_in_hexagon);sin(angles_in_hexagon)];
plot(point_in_hexagon(1,:),point_in_hexagon(2,:))