function tau=satellite_ctrl(in,P)
    theta1_d   = in(1);
    theta1     = in(2);
    theta2     = in(3);
    t          = in(4);
    
    % set persistent flag to initialize integrators and differentiators at
    % the start of the simulation
    persistent flag
    if t<P.Ts,
        flag = 1;
    else
        flag = 0;
    end
    
    % compute the desired angled angle using the outer loop control
    theta2_d = PID_th1(theta1_d,theta1,flag,P.kp_th1,P.ki_th1,P.kd_th1,P.A_th1,P.Ts,P.tau);
    % compute the force using the inner loop
    tau       = PID_th2(theta2_d,theta2,flag,P.kp_th2,P.ki_th2,P.kd_th2,P.tau_max,P.Ts,P.tau);
    
end

%------------------------------------------------------------
% PID control for position
function u = PID_th1(theta1_d,theta1,flag,kp,ki,kd,limit,Ts,tau)
    % declare persistent variables
    persistent integrator
    persistent theta1dot
    persistent error_d1
    persistent theta1_d1
    % reset persistent variables at start of simulation
    if flag==1,
        integrator  = 0;
        theta1dot   = 0;
        error_d1    = 0;
        theta1_d1   = 0;
    end
    
    % compute the error
    error = theta1_d-theta1;
    % update integral of error
    integrator = integrator + (Ts/2)*(error+error_d1);
    % update derivative of z
    theta1dot = (2*tau-Ts)/(2*tau+Ts)*theta1dot + 2/(2*tau+Ts)*(theta1-theta1_d1);
    % update delayed variables for next time through the loop
    error_d1  = error;
    theta1_d1 = theta1;

    % compute the pid control signal
    u_unsat = kp*error + ki*integrator - kd*theta1dot;
    u = sat(u_unsat,limit);
    
    % integrator anti-windup
    if ki~=0,
        integrator = integrator + Ts/ki*(u-u_unsat);
    end
end


%------------------------------------------------------------
% PID control for angle theta
function u = PID_th2(theta2_d,theta2,flag,kp,ki,kd,limit,Ts,tau)
    % declare persistent variables
    persistent integrator
    persistent theta2dot
    persistent error_d1
    persistent theta2_d1
    % reset persistent variables at start of simulation
    if flag==1,
        integrator   = 0;
        theta2dot    = 0;
        error_d1     = 0;
        theta2_d1    = 0;
    end
    
    % compute the error
    error = theta2_d-theta2;
    % update integral of error
    integrator = integrator + (Ts/2)*(error+error_d1);
    % update derivative of y
    theta2dot = (2*tau-Ts)/(2*tau+Ts)*theta2dot + 2/(2*tau+Ts)*(theta2-theta2_d1);
    % update delayed variables for next time through the loop
    error_d1 = error;
    theta2_d1 = theta2;

    % compute the pid control signal
    u_unsat = kp*error + ki*integrator - kd*theta2dot;
    u = sat(u_unsat,limit);
    
    % integrator anti-windup
    if ki~=0,
        integrator = integrator + Ts/ki*(u-u_unsat);
    end
end

%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end