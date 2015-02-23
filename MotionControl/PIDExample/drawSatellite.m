function drawSatellite(u,L,w)

    % process inputs to function
    theta1    = u(1);
    theta2    = u(2);
    t         = u(3);
    
    % define persistent variables 
    persistent base_handle
    persistent r_panel_handle
    persistent l_panel_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        track_width=3;
        plot([-2*L,2*L],[0,0],'k--'); % plot track
        hold on
        base_handle   = drawBase(theta1, theta2, L, w, [], 'normal');
        r_panel_handle  = drawRPanel(theta1, theta2, L, w, [], 'normal');
        l_panel_handle  = drawLPanel(theta1, theta2, L, w, [], 'normal');
        axis([-2*L, 2*L, -2*L, 2*L]);
    
        
    % at every other time step, redraw base and rod
    else 
        drawBase(theta1, theta2, L, w, base_handle);
        drawRPanel(theta1, theta2, L, w, r_panel_handle);
        drawLPanel(theta1, theta2, L, w, l_panel_handle);
    end
end

   
%
%=======================================================================
% drawBase
% draw the base of the pendulum
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawBase(theta1, theta2, L, w, handle, mode)

  % define points on base (without rotation)
  pts = [...
      w/2, -w/2;...
      w/2, w/2;...
      -w/2, w/2;...
      -w/2, -w/2;...
      ]';
  % define rotation matrix
  R = [cos(theta1), -sin(theta1); sin(theta1), cos(theta1)];
  % rotate points
  pts = R*pts;
  % break into X and Y components
  X = pts(1,:);
  Y = pts(2,:);

  if isempty(handle),
    handle = fill(X,Y,'b', 'EraseMode', mode);
    %handle = plot(X,Y,'m', 'EraseMode', mode);
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end
 
%
%=======================================================================
% drawRPanel
% draw the right panel
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawRPanel(theta1, theta2, L, w, handle, mode)

% define points on base (without rotation)
  pts = [...
      0, -w/6;...
      L, -w/6;...
      L, w/6;...
      0, w/6;...
      ]';
  % define rotation matrix
  R = [cos(theta1+theta2), -sin(theta1+theta2); sin(theta1+theta2), cos(theta1+theta2)];
  % rotate points
  pts = R*pts;
  % translate to correct position
  pts = pts + repmat([w/2*cos(theta1); w/2*sin(theta1)],1,4);
  % break into X and Y components
  X = pts(1,:);
  Y = pts(2,:);

  if isempty(handle),
    handle = fill(X, Y, 'g', 'EraseMode', mode);
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

%
%=======================================================================
% drawLPanel
% draw the left panel
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawLPanel(theta1, theta2, L, w, handle, mode)

% define points on base (without rotation)
  pts = [...
      0, -w/6;...
      L, -w/6;...
      L, w/6;...
      0, w/6;...
      ]';
  % define rotation matrix
  R = [cos(pi+theta1+theta2), -sin(pi+theta1+theta2); sin(pi+theta1+theta2), cos(pi+theta1+theta2)];
  % rotate points
  pts = R*pts;
  % translate to correct position
  pts = pts + repmat([-w/2*cos(theta1); -w/2*sin(theta1)],1,4);
  % break into X and Y components
  X = pts(1,:);
  Y = pts(2,:);

  if isempty(handle),
    handle = fill(X, Y, 'g', 'EraseMode', mode);
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

  