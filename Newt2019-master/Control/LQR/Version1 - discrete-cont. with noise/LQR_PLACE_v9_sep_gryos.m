%   Implementing a LQR controller for a two-gyroscope system.
%   The equations are slightly modified from the ones in
%   Ioannis report. A more detaild code can be found in Ioannis report.
%   
%   Author:     NEWT team
%   Course:     MF2059 - Mechatronic higher course
%   Date:       2019-10-05
%   Coach:      Lasse Hässler/ Didem
%   University: KTH - Royal Institure of Technology
clear all, close all, clc

%% ========================== Parameters ==================================
omega1  = 536.6;                % [rad/s]
omega2  = -536.6;               % [rad/s]
g       = 9.81;                 % [m/s^2]
% Gyro - flywheel inertia
Igz     = 0.062;                % [kg*m^2]
Igx     = 0.037;                % [kg*m^2]
Igy     = 0.037;                % [kg*m^2]
% Gyrp - cage inertia
Icx     = 0.095;                % [kg*m^2]
Icy     = 0.109;                % [kg*m^2]
Icz     = 0.154;                % [kg*m^2]
% Gyro-base inertia
Igbx    = 0;                    % [kg*m^2]
% Vehicle inertia
Ibx     = 10.756;               % [kg*m^2]
% Servo inertia
Isx     = 0.05;                 % [kg*m^2]
% Mass of flywheel
mg      = 8.848;                % [kg]
% Mass of cage
mc      = 8.636;                % [kg]
% Gyro-base mass & height of COG
mgb     = 0;                    % [kg]
hg      = 0.230;                % [m]
% Servo mass & height of COG
ms      = 2.15;                 % [kg]
hs      = 0.3;                  % [m]
%for i = 0.2:0.05:0.4
% Vehicle mass & height of COG
mb      = 100.651;              % [kg]
hb      = 0.21311;              % [m] 

%% =========================== EQUATIONS ==================================
% _______________________The equations for the system______________________
k1 = Icx + Igx;
k2 = Icy + Igy;
k3 = Icz + Igz;
k4 = Ibx + 2*Isx;
k5 = k1 - k3;
k6 = 2*hg*( mc+mg ) + hb*mb + 2*hs*ms;
k7 = 2*(hg^2)*( mc+mg ) + (hb^2)*mb + 2*(hs^2)*ms;

% ______________________Parameters for the state matrix____________________
a41 = ( g*k6 )/( k4 + k7 + 2*k1 );
a45 = ( -Igz*omega1 )/( k4 + k7 + 2*k1 );
a46 = ( -Igz*omega2 )/( k4 + k7 + 2*k1 );
a54 = ( omega1*Igz )/( k2 );
a64 = ( omega2*Igz )/( k2 );

% ______________________Parameter for the disturbance______________________
Fd = 100;
d41 = hb/( k4 + k7 + 2*k1 );
Dist_period = 50;

% __________________Parameters for the state noise vector__________________
xNoise_mean = 0;
xNoise_var = 0.01;

% _________________Parameters for the sensor noise vector__________________
sensNoise_mean = 0;
sensNoise_var = 0.01;

% ____________________Parameter for the input matrix_______________________
i51 = 1/k2;
i62 = 1/k2;

% ______________________IMU and encoder quantisizer________________________
imu_quant_ang = 0.005;
imu_quant_vel = 0.005;
enc_quant_ang = 0.022;
enc_quant_vel = 0.022;

%% =============================== MATRICES ==============================
% ______________________________State matrix_______________________________
A = [0 0 0 1 0 0
    0 0 0 0 1 0
    0 0 0 0 0 1
    a41 0 0 0 a45 a46
    0 0 0 a54 0 0
    0 0 0 a64 0 0];

% ______________________________Input matrix_______________________________
B = [0 0            % Theta
    0 0             % Alpha1
    0 0             % Alpha2
    0 0             % Theta_dot
    i51 0           % Alpha1_dot
    0 i62];         % Alpha2_dot

% ______________________State disturbance vector___________________________
V = [0              % Theta
    0               % Alpha1
    0               % Alpha2
    d41             % Theta_dot
    0               % Alpha1_dot
    0];             % Alpha2_dot

% __________________________State noise vector_____________________________
W = [0              % Theta
    0               % Alpha1
    0               % Alpha2
    0               % Theta_dot
    1               % Alpha1_dot
    -1];            % Alpha2_dot

% __________________________Sensor noise vector____________________________
S = [0              % Theta
    0               % Alpha1
    0               % Alpha2
    0               % Theta_dot
    0               % Alpha1_dot
    0];             % Alpha2_dot

% ___________________________Output matrix_________________________________
C = [1 0 0 0 0 0    % Theta
    0 1 0 0 0 0     % Alpha1
    0 0 1 0 0 0     % Alpha2
    0 0 0 1 0 0     % Theta_dot
    0 0 0 0 1 0     % Alpha1_dot
    0 0 0 0 0 1];   % Alpha2_dot

D = [0 0
    0 0
    0 0
    0 0
    0 0
    0 0];

%% ===================== INITIAL CONDITIONS ===============================
% _________________The initial conditions of the system____________________
roll_angle          = 0;        % [rad]
precession_angle1   = 0;        % [rad]
precession_angle2   = 0;        % [rad]
roll_rate           = 0;        % [rad/s]
precession_rate1    = 0;        % [rad/s]
precession_rate2    = 0;        % [rad/s]

Init_cond = [ roll_angle, precession_angle1, precession_angle2, roll_rate,...
    precession_rate1, precession_rate2 ];

init_roll_angle = 0;
init_roll_rate = 0;
%% ================== CONTROLLABILITY & OBSERVABILITY =====================
% ___________See if the system is controllable and observable______________
Controllability = ctrb( A, B );         % Controllability matrix
rank_ctrl = rank( Controllability );    % Rank of controllability matrix

Obs = obsv( A, C );                     % Observability matrix
rank_obs = rank( Obs );                 % Rank of observability matrix

if rank_ctrl == 6
    disp( 'System is controllable!' );
else
    disp( 'System is NOT controllable' );
end

if rank_obs == 6
    disp( 'System is observable!' );
else
    disp( 'System is NOT observable' );
end
fprintf( '\n' );

%% =========================== LQR ========================================
% Here we will do the lqr in continuous time. We are following Bryson's 
% rule in order to find the parameters in Q and R. This is an iterative 
% process to find desired characteristics of the system. Then we will 
% do the lqr to find the K-matrix (controller).
q11 = 2500;%1/( (deg2rad( 10 ) )^2 );                  % Theta
q22 = 3100;%1/( (deg2rad( 50 ) )^2 );                  % Alpha1
q33 = q22;                  % Alpha2
q44 = 1.4;%1/( 4.9^2 );                  % Theta_dot
q55 = 2.1;%1/( 3.7^2 );                  % Alpha1_dot
q66 = q55;                  % Alpha2_dot

r11 = 0.009;%1/( 100^2 );             % Control torque on flywheel 1
r22 = r11;             % Control torque on flywheel 2

Q_post = diag( [ q11 q22 q33 q44 q55 q66 ] ); 
Qc = C'*Q_post*C; 
Rc = diag( [ r11 r22 ] );

% _____________Good values for Q and R without quantisizer_________________
%________________________________________________BEST BUT OCCILATING_______
% q11:  18  !   20  !   20  !   25  !   q11=100 !   q11=2500    Keep "constant"
% q22:  28  !   45  !   40  !   18  !   q22=90  !   q22=3100    Keep "constant"
% q33:  -   !   -   !   -   !   -   !   q33=1   !   q33=3100    Keep "constant"
% q44:  2.9 !   3.9 !   3.9 !   0.35!   q44=1   !   q44=1.4     Keep "constant"
% q55:  3.7 !   2.7 !   2.7 !   1   !   q55=1   !   q55=2.1     Keep "constant"
% q66:  -   !   -   !   -   !   -   !   q66=1   !   q55=2.1     Keep "constant"
% r11:  50  !   45  !   140 !   100 !   r11=0.02!   r11=0.009   Change this one
% r22:  -   !   -   !   -   !   100 !   r22=0.02!   r22=0.009   Change this one
% ____________________________Conclusions:_________________________________
% Keep the q-parameters more or less the way they are now but change 
% the r-parameters. The change on r11 and r22 has more effect than changing
% the q-parameters. With Q > R -> more aggressive controller in order to 
% make the states go to zero. We then don't care about how much torque 
% we will use.
%
%
% _______________Good values for Q and R with quantisizer__________________
%________________________________________________BEST BUT OCCILATING_______
% q11=  2500 !  2500 !
% q22=  1100 !  1100 !
% q33=  2.4  !  2.4  !
% q44=  2.4  !  2.4  !
% q55=  3.1  !  7.1  !
% q66=  3.1  !  7.1  !
% r11=  0.04 !  0.06 !
% r22=  0.04 !  0.06 !
% Ts=   0.009!  0.007!
% ________________Calculate the controller through LQR_____________________
[ K, S, P ] = lqr( A, B, Qc, Rc );

n = length( P );
disp( '=========================' )
disp( '   Poles in cont. time   ' )
disp( '_________________________' )
fprintf( '  Number of poles are: %g \n', n )
disp( '_________________________' )
dataval = [ P ];
disp( dataval )
disp( '=========================' )

% _____________Print the ploes of the cont. system based on LQR____________
syscc = ss( A-B*K, B, C, D );                        % Cont. system
figure( 'Name', 'CONTINUOUS TIME: Poles', 'NumberTitle', 'off' )
set( gcf, 'Position', [500 200 900 700] )
pzmap( syscc )
title( 'CONTINUOUS TIME' );
grid on

%% ===================== Discrete time of lqr =============================
% Here the system is converted into discrete time from continuous time.
% The A and B matrices in the discrete time are extracted and we will 
% call 'dlqr' to get the K-matrix in discrete time. It is important to
% pick the right sampling time.
Ts = 0.001;                                   % Sampling time
Ts_min = ( 2*pi )/( real( max( abs( P ) ) )*10 );
fprintf( 'Minimum sampling time: %.4f \n', Ts_min );
Ts = input('Enter sampling time ');
sysc = ss( A, B, C, D );                    % System
sysdzoh = c2d( sysc, Ts, 'zoh' );           % Discretizing

G = sysdzoh.A;                              % A matrix in discrete time
H = sysdzoh.B;                              % B matrix in discrete time
[ Kd, Sd, Pd ] = dlqr( G, H, Qc, Rc );      % lqr in discrete time

n = length( Pd );
disp( '=========================' )
disp( ' Poles in discrete. time ' )
disp( '_________________________' )
fprintf( '  Number of poles are: %g \n', n )
disp( '_________________________' )
dataval = [ Pd ];
disp( dataval )
disp( '=========================' )

% ___________________Printing the discrete time poles______________________
figure( 'Name', 'DISCRETE TIME: Poles', 'NumberTitle', 'off' )
set( gcf, 'Position', [500 200 900 700] )
sysd = ss( G-H*Kd, H, C, D, Ts );
pzmap( sysd )
grid on
title( 'Poles of the discretized controller' )

%% ================= RUN THE SIMULINK MODEL DISCRETE ======================
model = sim( 'LQR_v9_two_separate_gyros_discrete' );

figure( 'Name', 'DISCRETE TIME: Angles', 'NumberTitle', 'off' )
set( gcf, 'Position', [500 200 900 700] )
plot( model.time, model.theta_d, model.time, model.alpha1_d,...
    model.time, model.alpha2_d, 'LineWidth', 2 )
title( 'Angle' );
legend( 'Theta', 'Alpha1', 'Alpha2' );
grid on

figure('Name', 'DISCRETE TIME: Torques', 'NumberTitle', 'off' )
set( gcf, 'Position', [500 200 900 700] )
plot( model.time, model.torque_d, 'LineWidth', 2 )
title( 'Control torque' );
legend( 'Control torque' );
grid on

figure( 'Name', 'DISCRETE TIME: Rates', 'NumberTitle', 'off' )
set( gcf, 'Position', [500 200 900 700] )
plot( model.time, model.theta_dot, model.time, model.alpha1_dot,...
    model.time, model.alpha2_dot, 'LineWidth', 2 )
title( 'Rate' );
legend( 'Roll rate', 'Alpha1 rate', 'Alpha2 rate' );
grid on

%% ================= RUN THE SIMULINK MODEL CONTINUOUS ====================
% sim( 'LQR_v9_two_separate_gyros' );
% 
% figure( 'Name', 'CONTINUOUS TIME: Angles', 'NumberTitle', 'off' )
% set( gcf, 'Position', [500 200 900 700] )
% plot( time, theta, time, alpha1,...
%     time, alpha2, 'LineWidth', 2 )
% title( 'Angle' );
% legend( 'Theta', 'Alpha1', 'Alpha2' );
% grid on
% 
% figure( 'Name', 'CONTINUOUS TIME: Torques', 'NumberTitle', 'off')
% set( gcf, 'Position', [500 200 900 700] )
% plot(time, torque, 'LineWidth', 2 )
% title( 'Control torque' );
% legend( 'Control torque' );
% grid on
% 
% figure( 'Name', 'CONTINUOUS TIME: Rates', 'NumberTitle', 'off' )
% set( gcf, 'Position', [500 200 900 700] )
% plot( time, theta_dot, time, alpha1_dot,...
%     time, alpha2_dot, 'LineWidth', 2 )
% title( 'Rate' );
% legend( 'Roll rate', 'Alpha1 rate', 'Alpha2 rate' );
% grid on

%% =========== RUN THE NON-LINEARIZED SIMULINK MODEL DISCRETE =============
% model_nl = sim( 'Version7_twogyros_nl' );
% 
% figure( 'Name', 'NON-LIN DISCRETE TIME: Angles', 'NumberTitle', 'off' )
% set( gcf, 'Position', [500 200 900 700] )
% plot( model_nl.time_nl, model_nl.theta_nl, '--', model_nl.time_nl, model_nl.alpha1_nl,...
%     '--', model_nl.time_nl, model_nl.alpha2_nl, '--', model_nl.time_nl, model_nl.theta_lin,...
%     model_nl.time_nl, model_nl.alpha1_lin,model_nl.time_nl, model_nl.alpha2_lin,...
%     'LineWidth', 1 )
% title( 'Angle' );
% legend( 'Theta_{NL}', 'Alpha1_{NL}', 'Alpha2_{NL}', 'Theta_{lin}', 'Alpha1_{lin}',...
%     'Alpha2_{lin}' );
% grid on
% 
% figure( 'Name', ' NON-LIN DISCRETE TIME: Torques', 'NumberTitle', 'off')
% set( gcf, 'Position', [500 200 900 700] )
% plot( model_nl.time_nl, model_nl.torque_nl, '--', model_nl.time_nl,...
%     model_nl.torque_lin, 'LineWidth', 1 )
% title( 'Control torque' );
% legend( 'Control torque_{NL}', 'Control torque_{lin}' );
% grid on
% 
% figure( 'Name', 'NON -LIN DISCRETE TIME: Rates', 'NumberTitle', 'off' )
% set( gcf, 'Position', [500 200 900 700] )
% plot( model_nl.time_nl, model_nl.theta_dot_nl, '--', model_nl.time_nl, model_nl.alpha1_dot_nl,...
%     '--', model_nl.time_nl, model_nl.alpha2_dot_nl, '--',...
%     model_nl.time_nl, model_nl.theta_dot_lin, model_nl.time_nl, model_nl.alpha1_dot_lin,...
%     model_nl.time_nl, model_nl.alpha2_dot_lin,'LineWidth', 1 )
% title( 'Rate' );
% legend( 'Roll_{NL} rate', 'Alpha1_{NL} rate', 'Alpha2_{NL} rate',...
%     'Roll_{lin} rate', 'Alpha_{lin} rate', 'Alpha2_{lin} rate' );
% grid on



