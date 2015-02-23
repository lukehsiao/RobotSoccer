% actual system parameters
AP.J1 = 0.01; % kg m^2
AP.m1 = 5;  % kg
AP.m2 = 0.5;% kg
AP.L1 = 1;  % m
AP.L2 = 4;  % m
AP.k = 0.15; % N m 
AP.b = 0.05; % N m s


% initial conditions
AP.theta10 = 0;
AP.theta20 = 0;
AP.theta1dot0 = 0;
AP.theta2dot0 = 0;

% parameters known to the controller
P.J1 = AP.J1;
P.m1 = AP.m1;
P.m2 = AP.m2;
P.L1 = AP.L1;
P.L2 = AP.L2;
P.k  = AP.k;  
P.b  = AP.b; 

% rename physical parameters
J11 = P.J1+2*P.m2*P.L1^2+2*P.m2*P.L2^2+2*P.m2*P.L1*P.L2;
J12 = 2*P.m2*P.L1*P.L2+2*P.m2*P.L2^2;
J22 = 2*P.J1+2*P.m2*P.L2^2;
Js = J11*J22-J12^2;
a1 = P.b*(J22+J12)/Js;
a2 = J22/Js;
a3 = a1;
a4 = 2*P.k*J12/Js;
a5 = P.b*(J11+J12)/Js;
%a6 = a4;
%a7 = a5;
a8 = -J12/Js;

% specs (tuning parameters) for inner loop
P.tau_max = 5; % N
A_th2 = 80*pi/180; % m
zeta_th2 = 2;%0.707;
P.kp_th2 = -P.tau_max/A_th2;
wn_th2 = sqrt(-a8*P.kp_th2);
P.kd_th2 = (a5-2*zeta_th2*wn_th2)/a8;
P.ki_th2 = 0;


% gains for outer loop
P.A_th1 = 40*pi/180;
zeta_th1 = 2;%0.707;
P.kp_th1 = -A_th1/A_th2;
wn_th1 = sqrt(a4*P.kp_th1);
P.kd_th1 = 2*zeta_th1*wn_th1/a4;
P.ki_th1 = 0;

P.Ts = 0.01; % sample rate of controller
P.tau = 0.05; % time constant for dirty derivative

