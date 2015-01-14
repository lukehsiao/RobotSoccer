% main control code - assumes full state knowledge
%
%
% Modified: 
%   2/11/2014 - R. Beard
%   2/18/2014 - R. Beard
%   2/24/2014 - R. Beard
%

%%%%%%%%%%%%%%%%%%% Main %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function v_c=controller_home_full_state(uu,P)

    % process inputs to function
    % robots - own team
    for i=1:P.num_robots,
        robot(:,i)   = uu(1+3*(i-1):3+3*(i-1));
    end
    NN = 3*P.num_robots;
    % robots - opponent
    for i=1:P.num_robots,
        opponent(:,i)   = uu(1+3*(i-1)+NN:3+3*(i-1)+NN);
    end
    NN = NN + 3*P.num_robots;
    % ball
    ball = [uu(1+NN); uu(2+NN)];
    NN = NN + 2;
    % score: own team is score(1), opponent is score(2)
    score = [uu(1+NN); uu(2+NN)];
    NN = NN + 2;
    % current time
    t      = uu(1+NN);
    
    v_c = strategy_switch_offense_and_defense(robot, ball, P, t);
end

%%%%%%%%%%%%%%%%%%%%%%%%%% Strategies %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function v_c = strategy_switch_offense_and_defense(robot, ball, P, t)
    persistent player_roles;
    persistent robot1_loc;
    persistent robot2_loc;
    
    % player role states
    normal = 0;
    reversed = 1;

    %initialize persistent variables
    if(t == 0)
        player_roles = normal;
        robot1_loc = robot(:,1);
        robot2_loc = robot(:,2);
    end
    
    % initialize player roles
    if(player_roles == normal)
        attacker = robot(:,1);
        defender = robot(:,2);
    else
        defender = robot(:,1);
        attacker = robot(:,2);
    end
    
    % Every second, compare robots location to its location 1 second ago
    if(mod(t, 1) == 0.0)
        old1_loc = robot1_loc;
        old2_loc = robot2_loc;
        robot1_loc = robot(:,1);
        robot2_loc = robot(:,2);
        dist1 = utility_calculate_distance(old1_loc(1), old1_loc(2), robot1_loc(1), robot1_loc(2));
        dist2 = utility_calculate_distance(old2_loc(1), old2_loc(2), robot2_loc(1), robot2_loc(2));
        
        % If neither robot has moved much in a while then switch roles
        if(dist1 < 0.15 && dist2 < 0.15)
            if(player_roles == normal)
                player_roles = reversed;
                defender = robot(:,1);
                attacker = robot(:,2);
            else
                player_roles = normal;
                attacker = robot(:,1);
                defender = robot(:,2);
            end
        end
    end
    
    
    % if the ball gets behind the attacker then switch roles
    if(ball(1) < (attacker(1)-((P.robot_radius)/2)))
        if(player_roles == normal)
            player_roles = reversed;
            defender = robot(:,1);
            attacker = robot(:,2);
        else
            player_roles = normal;
            attacker = robot(:,1);
            defender = robot(:,2);
        end
    end
    
    defense = 0;
    offense = 1;
    playtype = offense;
    
    % If the ball gets close to mid-field then go on defense
    if(ball(1) < (P.field_width/8))
        playtype = defense;
    end
    
    % play chosen based on role state and if we are on offense or defense
    if(player_roles == normal)
        if(playtype == offense)
            v1 = play_rush_goal(attacker, ball, P);
            v2 = skill_follow_ball_on_line(defender, ball, 0, P);
        else
            v1 = play_rush_goal(attacker, ball, P);
            v2 = skill_guard_goal(defender, ball, P);
        end
    else
        if(playtype == offense)
            v2 = play_rush_goal(attacker, ball, P);
            v1 = skill_follow_ball_on_line(defender, ball, 0, P);
        else
            v2 = play_rush_goal(attacker, ball, P);
            v1 = skill_guard_goal(defender, ball, P);
        end
    end
    
    % output velocity commands to robots
    v1 = utility_saturate_velocity(v1,P);
    v2 = utility_saturate_velocity(v2,P);
    v_c = [v1; v2];
end

%%%%%%%%%%%%%%%%%%%%%%%%% Plays %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%-----------------------------------------
% play - rush goal
%   - go to position behind ball
%   - if ball is between robot and goal, go to goal
% NOTE:  This is a play because it is built on skills, and not control
% commands.  Skills are built on control commands.  A strategy would employ
% plays at a lower level.  For example, switching between offense and
% defense would be a strategy.
function v = play_rush_goal(robot, ball, P)
  
  % normal vector from ball to goal
  n = P.goal-ball;
  n = n/norm(n);
  % compute position 10cm behind ball, but aligned with goal.
  position = ball - 0.2*n;
    
  if norm(position-robot(1:2))<.21,
      v = skill_go_to_point_angle_corrected(ball, robot, P.goal, P);
  else
      v = skill_go_to_point(robot, position, P);
  end

end

%%%%%%%%%%%%%%%%%%%%%%% Skills %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%-----------------------------------------
% skill - follow ball on line
%   follows the y-position of the ball, while maintaining x-position at
%   x_pos.  Angle always faces the goal.

function v=skill_follow_ball_on_line(robot, ball, x_pos, P)

    % control x position to stay on current line
    vx = -P.control_k_vx*(robot(1)-x_pos);
    
    % control y position to match the ball's y-position
    vy = -P.control_k_vy*(robot(2)-ball(2));

    % control angle to -pi/2
    theta_d = atan2(P.goal(2)-robot(2), P.goal(1)-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    
    v = [vx; vy; omega];
end

%------------------------------------------
% skill - follow ball on line in front of goal, never leaving goal
%   follows the y-position of the ball, while maintaining x position in
%   front of the goal. Angle always faces the ball. Does not leave the
%   area of the goal.

function v=skill_guard_goal(robot, ball, P)
    % control x position to stay on -15/16 the field length
    vx = -P.control_k_vx*(robot(1)-(-15*P.field_width/16));
    
    % control y position to match the ball's y-position while ball is
    % within the goal. Otherwise, stay at edges.
    if ball(2) > P.field_width/6
        vy = -P.control_k_vy*(robot(2)-(P.field_width/6));
    elseif ball(2) < -P.field_width/6
        vy = -P.control_k_vy*(robot(2)-(-P.field_width/6));
    else
        vy = -P.control_k_vy*(robot(2)-ball(2));
    end

    % control angle to face ball, but not exceeding +/- 90 degrees.
    theta_d = atan2(ball(2)-robot(2), ball(1)-robot(1));
    if theta_d >= pi/2
       theta_d = pi/2.25;  % angle pushes ball out (more effective)
    elseif theta_d <= -pi/2
        theta_d = -pi/2.25;
    end
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    v = [vx; vy; omega];
end

%-----------------------------------------
% skill - go to point
%   follows the y-position of the ball, while maintaining x-position at
%   x_pos.  Angle always faces the goal.

function v=skill_go_to_point(robot, point, P)

    % control x position to stay on current line
    vx = -P.control_k_vx*(robot(1)-point(1));
    
    % control y position to match the ball's y-position
    vy = -P.control_k_vy*(robot(2)-point(2));

    % control angle to -pi/2
    theta_d = atan2(P.goal(2)-robot(2), P.goal(1)-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    
    v = [vx; vy; omega];
end

function v=skill_go_to_point_angle_corrected(ball, robot, point, P)

    % control x position to stay on current line
    vx = -P.control_k_vx*(robot(1)-point(1));
    
    % control y position to match the ball's y-position
    vy = -P.control_k_vy*(robot(2)-point(2));
    
    y_delta = robot(2) - ball(2);

    % control angle to -pi/2
    theta_d = atan2((P.goal(2)-(y_delta*60))-ball(2), P.goal(1)-ball(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    
    v = [vx; vy; omega];
end

function roles=skill_switch_roles(player_roles, myparams, robot)
    if(player_roles == myparams.normal)
        player_roles = myparams.reversed;
        defender = robot(:,1);
        attacker = robot(:,2);
    else
        player_roles = myparams.normal;
        attacker = robot(:,1);
        defender = robot(:,2);
    end
    roles(1) = player_roles;
    roles(2) = attacker(3);
    roles(3) = attacker(2);
    roles(4) = attacker(1);
    roles(5) = defender(3);
    roles(6) = defender(2);
    roles(7) = defender(1);
end

function v=skill_go_to_point_avoid_obstacles(robot, point, P)
    % control x position to stay on current line
    vx = -P.control_k_vx*(robot(1)-point(1));
    
    % control y position to match the ball's y-position
    vy = -P.control_k_vy*(robot(2)-point(2));

    % control angle to -pi/2
    theta_d = atan2(P.goal(2)-robot(2), P.goal(1)-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    
    v = [vx; vy; omega];
end

%%%%%%%%%%%%%%%%%%% Utilities %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

function distance = utility_calculate_distance(item1_x, item1_y, item2_x, item2_y)
    distance = sqrt((item1_x - item2_x)^2 + (item1_y - item2_y)^2);
end


%------------------------------------------
% utility - saturate_velocity
% 	saturate the commanded velocity 
%
function v = utility_saturate_velocity(v,P)
    if v(1) >  P.robot_max_vx,    v(1) =  P.robot_max_vx;    end
    if v(1) < -P.robot_max_vx,    v(1) = -P.robot_max_vx;    end
    if v(2) >  P.robot_max_vy,    v(2) =  P.robot_max_vy;    end
    if v(2) < -P.robot_max_vy,    v(2) = -P.robot_max_vy;    end
    if v(3) >  P.robot_max_omega, v(3) =  P.robot_max_omega; end
    if v(3) < -P.robot_max_omega, v(3) = -P.robot_max_omega; end
end

%ESTIMATEBALLVELOCITY Estimate the balls velocity based on current postion
%   Take the diferential of the balls current and previous position. v(1) 
%   is the balls velocity, v(2) is the current direction of the ball, v(3)
%   indicates if the estimation is valid (1 for yes, 0 for no);
function v = utility_get_ball_info(ball, P)
    % Persisitent Variables
    persistent first_run;
    persistent position_x_prev;
    persistent position_y_prev;
    persistent velocity_x;
    persistent velocity_y;
    persistent magnitude_prev;
    persistent direction_prev;
    
    %parsing ball
    position_x = ball(1);
    position_y = ball(2);
    
    %initialize variables
    if isempty(first_run),
        position_x_prev = position_x;
        position_y_prev = position_y;
        velocity_x = 0;
        velocity_y = 0;
        magnitude_prev = 0;
        direction_prev = 0;
        first_run = 0;
    end
    
    % Estiamate the x and y velocity of the ball
    tau = 1/(30*2*pi);
    velocity_x = (2*tau -P.control_sample_rate)/...
        (2*tau+P.control_sample_rate)*velocity_x +...
        2/(2*tau+P.control_sample_rate)*(position_x - position_x_prev);
    velocity_y = (2*tau -P.control_sample_rate)/...
        (2*tau+P.control_sample_rate)*velocity_y +...
        2/(2*tau+P.control_sample_rate)*(position_y - position_y_prev);
    
    % Calculate Velocity
    magnitude = norm([velocity_x;velocity_y]);
    
    % Calculate Direction
    direction = atan2(velocity_y,velocity_x);
    
    % validate the estimated data
    estimated_magnitude = -P.ball_mu*magnitude_prev*...
        P.control_sample_rate + magnitude_prev;
    magdiff = abs(magnitude - estimated_magnitude);
    dirdiff = abs(direction - direction_prev);
        
    information_valid = 1;
    
    if dirdiff > .01 * abs(direction_prev) || ...
            magdiff > .01 * abs(magnitude_prev),
        information_valid = 0;
    end
    
    % Save variables
    position_x_prev = position_x;
    position_y_prev = position_y;
    magnitude_prev = magnitude;
    direction_prev = direction;
    
    % Set output
    v(1) = magnitude;
    v(2) = direction;
    v(3) = information_valid;
end

%%%%%%%%%%%%%%%%%%%% Other %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function var=defines()
    normal = 0;
    reversed = 1;
end