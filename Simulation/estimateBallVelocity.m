function v = estimateBallVelocity(ball, P)
%ESTIMATEBALLVELOCITY Estimate the balls velocity based on current postion
%   Take the diferential of the balls current and previous position. v(1) 
%   is the balls velocity, v(2) is the current direction of the ball, v(3)
%   indicates if the estimation is valid (1 for yes, 0 for no);
    
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

