% Parameter file for robot soccer simulator
%
% Modified: 2/11/2014 - R. Beard
%


% number of robots per team
P.num_robots = 2;

% field characteristics
P.field_length  = 3.048; % meters (5 ft)
P.field_width   = 1.52; % meters (10 ft)
P.field_color = [16,92,1]/256;
P.home_team_color = 'b';
P.away_team_color = 'g';
P.ball_color = 'y';
P.goal_width = P.field_width/3;
P.goal_color = [252 148 3]/256;
P.display_rate = 0.1;

% marker locations  % in home team coordinate systems
P.marker = [...
      -P.field_length/2, P.field_width/2;...
      0, P.field_width/2;...
      P.field_length/2, P.field_width/2;...
      P.field_length/2, -P.field_width/2;...
      0, -P.field_width/2;...
      -P.field_length/2, -P.field_width/2;...
      ]';
P.num_markers = size(P.marker,2);
P.marker_radius = 0.04;
P.marker_color = [0 150 148]/256;
  
% constants that govern ball dynamics
P.ball_radius = 0.03;
P.ball_mu = 0.1;  % coefficient of friction for ball
P.ball_spring = 500;% spring constant that models wall and robot interactions

% characteristics of camera
P.camera_min_range = .15; 
P.camera_max_range = 3;
P.camera_fov = 63*pi/180;
P.camera_out_of_range = -999;
P.camera_frame_rate = 0.03;

% robot parameters and geometry
P.wheel_radius = 0.03; % m (this is a guess)
P.robot_radius = 0.1;  % 4 inches
P.robot_max_vx = .8; % (m/s) max speed in x direction
P.robot_max_vy = .5; % (m/s) max speed in y direction
P.robot_max_omega = 2*pi; % 360 degrees/sec
% the geometry assumes 60 degree equally distributed wheels at distance 5cm
% from center
phi = 60*pi/180;
r1 = P.robot_radius*[cos(phi); sin(phi)];
r2 = P.robot_radius*[-cos(phi); sin(phi)];
r3 = P.robot_radius*[0;1];
s1 = [-sin(phi); cos(phi)];
s2 = [-sin(phi); -cos(phi)];
s3 = [1; 0];
% kinematic matrix relating wheel velocity to body velociy
P.M3 = 1/P.wheel_radius*[...
    s1(1), s1(2), (s1(2)*r1(1)-s1(1)*r1(2));...
    s2(1), s2(2), (s2(2)*r2(1)-s2(1)*r2(2));...
    s3(1), s3(2), (s3(2)*r3(1)-s3(1)*r3(2))];
P.M3inv = inv(P.M3);

% initial states for robots and ball
P.home_team_initial_configurations = [...
    -P.field_length/6; 0;  0;... % robot 1, position and orientation
    -P.field_length/3; 0;  0;...  % robot 2, position and orientation
    ];
P.away_team_initial_configurations = [...
    -P.field_length/6; 0; 0;... % robot 1, position and orientation
    -P.field_length/3; 0; 0;...   % robot 2, position and orientation
    ];
P.ball_initial_position = [ 0; 0];     % in frame of home team
P.ball_initial_velocity = [0.3; 0.5];  % in frame of home team

% goal positions
P.goal = [P.field_length/2; 0];

% controller gains
P.control_sample_rate = 0.01;
P.control_k_vx  = 5;  % gain for proportional control of x-position
P.control_k_vy  = 5;  % gain for proportional control of y-position
P.control_k_phi = 2;  % gain for proportional angle control

% observer gains
P.observer_Q_self = 100*eye(3);
P.observer_R_range = .1;
P.observer_R_bearing = 1*pi/180;
  
 

    

