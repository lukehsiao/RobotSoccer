% draw robot soccer field, robots, and ball
%
% Modified: 
%   2/11/2014 - R. Beard
%   2/18/2014 - R. Beard
%
function drawEnvironment(uu,P)

    % process inputs to function
    % robots for home team  (shooting left to right)
    for i=1:P.num_robots,
        rob_home(i).x   = uu(1+3*(i-1));
        rob_home(i).y   = uu(2+3*(i-1));
        rob_home(i).phi = uu(3+3*(i-1));
    end
    % robots for away team (shooting right to left)
    for i=1:P.num_robots,
        rob_away(i).x   = uu(1+3*(i-1)+3*P.num_robots);
        rob_away(i).y   = uu(2+3*(i-1)+3*P.num_robots);
        rob_away(i).phi = uu(3+3*(i-1)+3*P.num_robots);
    end
    % ball
    ball.x = uu(1+6*P.num_robots);
    ball.y = uu(2+6*P.num_robots);
    % state estimate and covariance
    NN = 2+6*P.num_robots;
    % score
    score = uu(1+NN:2+NN);
    NN = NN + 2;
    % current time
    t      = uu(1+NN);
    
    % define persistent variables 
    persistent rob_home_handle;  % figure handle for home team
    persistent rob_away_handle;  % figure handle for away team
    persistent ball_handle;      % figure handle for ball
    persistent score_handle;     % figure handle for scores
    persistent score_old;        % old score handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        drawField(P);
        score_handle = drawScore(0,0,[],P);
        score_old = score;
        for i=1:P.num_robots,
            rob_home_handle(i) = drawRobotHome(rob_home(i),[],'normal',P);
            rob_away_handle(i) = drawRobotAway(rob_away(i),[],'normal',P);
        end
        ball_handle = drawBall(ball,[],'normal',P);
%        xhat_handle = drawStateEstimate(xhat,S,[],'normal',P);
                
    % at every other time step, redraw MAV
    else 
        for i=1:P.num_robots,
            drawRobotHome(rob_home(i),rob_home_handle(i),'normal',P);
            drawRobotAway(rob_away(i),rob_away_handle(i),'normal',P);
        end
        drawBall(ball,ball_handle,'normal',P);
%        drawStateEstimate(xhat,S,xhat_handle,'normal',P);
        if norm(score-score_old)~=0,
            score_old = score;
            drawScore(score(1),score(2),score_handle,P);
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawRobotHome(rob,handle,mode,P)
    th = [0, -pi/8, -pi/4, -3*pi/8, -pi/2, -5*pi/8, -3*pi/4, -7*pi/8, -pi, 0];
    pts = P.robot_radius*[sin(th); cos(th)];
    R = [cos(rob.phi), -sin(rob.phi); sin(rob.phi), cos(rob.phi)];
    pts = R*pts;
    pts = pts + repmat([rob.x; rob.y],1,size(pts,2));

    if isempty(handle),
        handle = fill(pts(1,:), pts(2,:),P.home_team_color,'EraseMode', mode);
    else
        set(handle,'XData',pts(1,:),'YData',pts(2,:));
        drawnow
    end
  
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawRobotAway(rob,handle,mode,P)
    th = [0, -pi/8, -pi/4, -3*pi/8, -pi/2, -5*pi/8, -3*pi/4, -7*pi/8, -pi, 0];
    pts = P.robot_radius*[sin(th); cos(th)];
    R = [cos(rob.phi+pi), -sin(rob.phi+pi); sin(rob.phi+pi), cos(rob.phi+pi)];
    pts = R*pts;
    pts = pts - repmat([rob.x; rob.y],1,size(pts,2));

    if isempty(handle),
        handle = fill(pts(1,:), pts(2,:),P.away_team_color,'EraseMode', mode);
    else
        set(handle,'XData',pts(1,:),'YData',pts(2,:));
        drawnow
    end
  
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawBall(ball,handle,mode,P)
    
    phi = [0:.1:2*pi];
    pts = P.ball_radius*[cos(phi); sin(phi)];
    pts = pts + repmat([ball.x; ball.y],1,size(pts,2));

    if isempty(handle),
        handle = fill(pts(1,:), pts(2,:), P.ball_color, 'EraseMode', mode);
    else
        set(handle,'XData',pts(1,:),'YData',pts(2,:));
        drawnow
    end
  
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Display Score
function handle = drawScore(homescore, awayscore,handle,P)

    if isempty(handle),
        handle(1)=text(-P.field_length/4,P.field_width/2+P.field_width/12,sprintf('Home: %d',homescore),'FontSize',24);
        handle(2)=text(P.field_length/4,P.field_width/2+P.field_width/12,sprintf('Away: %d',awayscore),'FontSize',24);
    else
        set(handle(1),'String',sprintf('Home: %d',homescore));
        set(handle(2),'String',sprintf('Away: %d',awayscore));
        drawnow
    end
end 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawStateEstimate(xhat,S,handle,mode,P)
    sig = 3;
    phi = [0:.1:2*pi];
    Sinv = inv(S);
    M = sqrtm(Sinv(1:2,1:2));
    pts = sig*inv(M)*[cos(phi); sin(phi)]/10+repmat(xhat(1:2),1,length(phi));

    if isempty(handle),
        handle(1) = plot(xhat(1),xhat(2),'rx', 'EraseMode', mode);
        handle(2) = plot(pts(1,:), pts(2,:), 'r', 'EraseMode', mode);
    else
        set(handle(1),'XData',xhat(1),'YData',xhat(2));
        set(handle(2),'XData',pts(1,:),'YData',pts(2,:));
        drawnow
    end
  
end 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function drawField(P)
    W = P.field_width;
    L = P.field_length;
    G = P.goal_width;
    D = L/10;
    phi = [0:.1:2*pi,0];

    % field and display characteristics
    axis([-L/2-D,L/2+D,-W/2-D,W/2+D]);
    X = [-L/2,L/2,L/2,-L/2];
    Y = [W/2,W/2,-W/2,-W/2];
    fill(X,Y,P.field_color); 
    hold on
    axis off
    axis image
    
    % lines on the field
    plot([0,0],[-W/2,W/2],'k') % center line
    plot([L/2,L/2-L/10,L/2-L/10,L/2],[-W/3,-W/3,W/3,W/3],'k') % right goal box
    plot([-L/2,-L/2+L/10,-L/2+L/10,-L/2],[-W/3,-W/3,W/3,W/3],'k') % left goal box
    plot(W/4*cos(phi),W/4*sin(phi),'k') % center circle
    
    % goals
    fill([L/2,L/2-L/40,L/2-L/40,L/2],[-G/2,-G/2,G/2,G/2],P.goal_color) % left goal
    fill([-L/2,-L/2+L/40,-L/2+L/40,-L/2],[-G/2,-G/2,G/2,G/2],P.goal_color) % right goal
    
    % markers
    % NOTE: this is a hack because I am not using P.marker.  The
    % vision system sees markers defined in P.marker and not here.  Need to
    % fix.
    fill(-L/2+P.marker_radius*cos(phi),W/2+P.marker_radius*sin(phi),P.marker_color)
    text(-L/2-.05,W/2+.05,'1')
    fill(P.marker_radius*cos(phi),W/2+P.marker_radius*sin(phi),P.marker_color)
    text(0,W/2+.05,'2')
    fill(L/2+P.marker_radius*cos(phi),W/2+P.marker_radius*sin(phi),P.marker_color)
    text(L/2+.05,W/2+.05,'3')
    fill(L/2+P.marker_radius*cos(phi),-W/2+P.marker_radius*sin(phi),P.marker_color)
    text(L/2+.05,-W/2-.05,'4')
    fill(P.marker_radius*cos(phi),-W/2+P.marker_radius*sin(phi),P.marker_color)
    text(0,-W/2-.05,'5')
    fill(-L/2+P.marker_radius*cos(phi),-W/2+P.marker_radius*sin(phi),P.marker_color)
    text(-L/2-.05,-W/2-.05,'6')
    
  
end 

 
  

  