% observer for robot A-1
% xhat is the estimate of the state of the robot: x=[rx; ry; phi], and S is
% the error covariance
%
% for now the observer only uses the range and bearing to the markers to
% estimate own position
%
% Modified: 2/11/2014 - R. Beard
%
function out=observer(uu,P)

    % put vision data into structures to make it easier to process
    vision.ball     = uu(1:2);                     NN = 2;
    vision.ownteam  = uu(1+NN:2*P.num_robots+NN);  NN = NN + 2*P.num_robots;
    vision.opponent = uu(1+NN:2*P.num_robots+NN);  NN = NN + 2*P.num_robots;
    vision.marker   = reshape(uu(1+NN:2*P.num_markers+NN),2,P.num_markers); NN = NN + 2*P.num_markers;
    % commanded velocity
    velocity = uu(1+NN: 3+NN); NN = NN + 3;
    % current simulation time
    t = uu(1+NN);

    persistent xhat
    persistent S
    
    if t==0,
        xhat = [-P.field_length/6; 0; -pi/2];
        S = diag([P.field_length/12; P.field_width/12; pi/20]);
    end
    
    % estimate between measurements
    N = 10;
    for i=1:N,
        xhat = xhat + (P.control_sample_rate/N)*(velocity);
        S = S + (P.control_sample_rate/N)*(P.observer_Q_self);
    end
 
    % measurement updates
    for i=1:P.num_markers,
        % range measurement
        if vision.marker(1,i)~=P.camera_out_of_range,
            rho = P.marker(:,i)-xhat(1:2);
            Rho = norm(rho);
            h = Rho;
            H = [-rho(1), -rho(2), 0]/Rho;
            L = S*H'/(P.observer_R_range+H*S*H');
            S = (eye(3)-L*H)*S;
            xhat = xhat + L*(vision.marker(1,i)-h);
        end
        % bearing measurement
        if vision.marker(2,i)~=P.camera_out_of_range,
            rho = P.marker(:,i)-xhat(1:2);
            Rho = norm(rho);
            phi = xhat(3);
            h = asin((rho(2)*cos(phi)-rho(1)*sin(phi))/Rho);
            H = sign(rho(1)*cos(phi)+rho(2)*sin(phi))...
                *[rho(2)/(Rho^2), -rho(1)/(Rho^2), -1];
            L = S*H'/(P.observer_R_bearing+H*S*H');
            S = (eye(3)-L*H)*S;
            xhat = xhat + L*(vision.marker(2,i)-h);
        end
        
    end   
  
    % create output
    out = [xhat; reshape(S,9,1)];
end
  

  