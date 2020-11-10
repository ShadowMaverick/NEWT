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

%% =========================== Parameters =================================
omega1  = 536.6;                % [rad/s]
omega2  = -536.6;               % [rad/s]
g       = 9.81;                 % [m/s^2]

% ________________________Gyro - flywheel inertia__________________________
Igz     = 0.062;                % [kg*m^2]
Igx     = 0.037;                % [kg*m^2]
Igy     = 0.037;                % [kg*m^2]

% ___________________________Gyro - cage inertia___________________________
Icx     = 0.095;                % [kg*m^2]
Icy     = 0.109;                % [kg*m^2]
Icz     = 0.154;                % [kg*m^2]

% ____________________________Gyro-base inertia____________________________
Igbx    = 0;                    % [kg*m^2]

% _____________________________Vehicle inertia_____________________________
Ibx     = 10.756;               % [kg*m^2]

% ______________________________Servo inertia______________________________
Isx     = 0.05;                 % [kg*m^2]

% _____________________________Mass of flywheel____________________________
mg      = 8.848;                % [kg]

% _______________________________Mass of cage______________________________
mc      = 8.636;                % [kg]

% _____________________Gyro-base mass & height of COG______________________
mgb     = 0;                    % [kg]
hg      = 0.230;                % [m]

% _______________________Servo mass & height of COG________________________
ms      = 2.15;                 % [kg]
hs      = 0.3;                  % [m]

% ______________________Vehicle mass & height of COG_______________________
mb      = 100.651;              % [kg]
hb      = 0.21311;              % [m] 

%% =========================== EQUATIONS ==================================
% ________________________Equations for the system_________________________
k1 = Icx + Igx;
k2 = Icy + Igy;
k3 = Icz + Igz;
k4 = Ibx + 2*Isx;
k5 = k1 - k3;
k6 = 2*hg*( mc + mg ) + hb*mb + 2*hs*ms;
k7 = 2*( hg^2 )*( mc + mg ) + ( hb^2 )*mb + 2*( hs^2 )*ms;

% ______________________Parameters for the state matrix____________________
a41 = ( g*k6 )/( k4 + k7 + 2*k1 );
a45 = ( -Igz*omega1 )/( k4 + k7 + 2*k1 );
a46 = ( -Igz*omega2 )/( k4 + k7 + 2*k1 );
a54 = ( omega1*Igz )/( k2 );
a64 = ( omega2*Igz )/( k2 );

% _____________________Parameters for the disturbance______________________
Fd = 600; %480;                     % Impulse force disturbance
F_d = 11; %11;                       % Contant force disturbance
d41 = hb/( k4 + k7 + 2*k1 );        % Constant in V matrix
Dist_period = 10;                   % The frequency of disturbance force

% ____________________Parameters for the input matrix______________________
i51 = 1/k2;
i62 = 1/k2;

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

% ___________________________Output matrix_________________________________
C = [1 0 0 0 0 0    % Theta
    0 1 0 0 0 0     % Alpha1
    0 0 1 0 0 0     % Alpha2
    0 0 0 1 0 0     % Theta_dot
    0 0 0 0 1 0     % Alpha1_dot
    0 0 0 0 0 1];   % Alpha2_dot

% _________________________Feedforward matrix______________________________
D = [0 0            % Theta
    0 0             % Alpha1
    0 0             % Alpha2
    0 0             % Theta_dot
    0 0             % Alpha1_dot
    0 0];           % Alpha2_dot

%% ====================== INITIAL CONDITIONS ==============================
% ___________________Initial conditions of the system______________________
roll_angle          = pi/180*15;        % [rad]
precession_angle1   = 0;                % [rad]
precession_angle2   = 0;                % [rad]
roll_rate           = 0;                % [rad/s]
precession_rate1    = 0;                % [rad/s]
precession_rate2    = 0;                % [rad/s]

Init_cond = [ roll_angle, precession_angle1, precession_angle2, roll_rate,...
    precession_rate1, precession_rate2 ];

init_roll_angle     = pi/180*15;            % Initial roll angle in radians
init_roll_rate      = 0;
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

%% ============================= LQR ======================================
% Here we will do the lqr in continuous time. We are following Bryson's 
% rule in order to find the parameters in Q and R. This is an iterative 
% process to find desired characteristics of the system. Then we will 
% do the lqr to find the K-matrix (controller).
% --------
% q11 = 1/((deg2rad( 15 ))^2);% 2500;%1/( (deg2rad( 10 ) )^2 );                  % Theta
% q22 = 1/((deg2rad( 80 ))^2);%3100;%1/( (deg2rad( 50 ) )^2 );                  % Alpha1
% q33 = q22;                  % Alpha2
% q44 = 1/( 2.6^2 );%1.4;%1/( 4.9^2 );                  % Theta_dot
% q55 = 1/( 3.1^2 );%2.1;%1/( 3.7^2 );                  % Alpha1_dot
% q66 = q55;                  % Alpha2_dot
% 
% r11 = 1/70^2;%0.001;%1/( 100^2 );             % Control torque on flywheel 1
% r22 = r11;             % Control torque on flywheel 2
% 
% Q_post = diag( [ q11 q22 q33 q44 q55 q66 ] ); 
% Qc = C'*Q_post*C; 
% Rc = diag( [ r11 r22 ] );
% --------
QQ = [1/deg2rad(20) 0 0 0 0 0
    0 1/deg2rad(50) 0 0 0 0
    0 0 1/deg2rad(50) 0 0 0
    0 0 0 1/9.3 0 0
    0 0 0 0 1/4.0 0
    0 0 0 0 0 1/4.0];

Q_pen = diag( [105 27 27 12.7 94.5 94.5] );

RR = [1/70 0
    0 1/70];

R_pen = diag( [1.7 1.7] );

Qc = QQ'*Q_pen*QQ;
Rc = RR'*R_pen*RR;

% _____________Good values for Q and R without quantisizer_________________
%__________________________BEST BUT OCCILATING_____________________________
% q11:  20  !   20  !   25  !   q11=100 !   q11=2500    Keep "constant"
% q22:  45  !   40  !   18  !   q22=90  !   q22=3100    Keep "constant"
% q33:  -   !   -   !   -   !   q33=1   !   q33=3100    Keep "constant"
% q44:  3.9 !   3.9 !   0.35!   q44=1   !   q44=1.4     Keep "constant"
% q55:  2.7 !   2.7 !   1   !   q55=1   !   q55=2.1     Keep "constant"
% q66:  -   !   -   !   -   !   q66=1   !   q55=2.1     Keep "constant"
% r11:  45  !   140 !   100 !   r11=0.02!   r11=0.009   Change this one
% r22:  -   !   -   !   100 !   r22=0.02!   r22=0.009   Change this one
% ____________________________Conclusions:_________________________________
% Keep the q-parameters more or less the way they are now but change 
% the r-parameters. The change on r11 and r22 has more effect than changing
% the q-parameters. With Q > R -> more aggressive controller in order to 
% make the states go to zero. We then don't care about how much torque 
% we will use.
%
% _______________Good values for Q and R with quantisizer__________________
%___________________________BEST BUT OCCILATING____________________________
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
% syscc = ss( A-B*K, B, C, D );                        % Cont. system
% figure( 'Name', 'CONTINUOUS TIME: Poles', 'NumberTitle', 'off' )
% set( gcf, 'Position', [500 200 900 700] )
% pzmap( syscc )
% title( 'CONTINUOUS TIME' );
% grid on

%% ===================== Discrete time of LQR =============================
% Here the system is converted into discrete time from continuous time.
% The A and B matrices in the discrete time are extracted and we will 
% call 'dlqr' to get the K-matrix in discrete time. It is important to
% pick the right sampling time.
Ts_min = ( 2*pi )/( real( max( abs( P ) ) )*10 );
%fprintf( 'Minimum sampling time: %.4f \n', Ts_min );
%Ts = input('Enter sampling time ');
Ts = Ts_min;
sysc = ss( A, B, C, D );                    % System
sysdzoh = c2d( sysc, Ts, 'zoh' );           % Discretizing

G = sysdzoh.A;                              % A matrix in discrete time
H = sysdzoh.B;                              % B matrix in discrete time
[ Kd, Sd, Pd ] = dlqr( G, H, Qc, Rc );      % LQR in discrete time

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
% figure('Name', 'DISCRETE TIME: Poles', 'NumberTitle', 'off' )
% set( gcf, 'Position', [500 200 900 700] )
% sysd = ss( G-H*Kd, H, C, D, Ts );
% pzmap( sysd )
% grid on
% title( 'Poles of the discretized controller' )

%% =========================== ALL PLOTS ==================================

% ________________________Run simulink models______________________________
model = sim( 'LQR_v9_two_separate_gyros_discrete' );
model_nl = sim( 'Version7_twogyros_nl' );

% ____________________________Roll angle___________________________________
figure( 'Name', 'Roll angle', 'NumberTitle', 'off' )
set( gcf, 'Position', [500 200 900 700] )
plot( model_nl.time_nl, model_nl.theta_nl, 'r', model.time, model.theta_d )
title( 'Roll angle' );
legend( 'Non- linear', 'Linear' );
grid on

% ____________________________Roll rate____________________________________
figure( 'Name', 'Roll rate', 'NumberTitle', 'off' )
set( gcf, 'Position', [500 200 900 700] )
plot( model_nl.time_nl, model_nl.theta_dot_nl, model.time, model.theta_dot )
title( 'Roll rate' );
legend( 'Non- linear', 'Linear' );
grid on

% ________________________Precession angle_________________________________
figure( 'Name', 'Precession angles', 'NumberTitle', 'off' )
set( gcf, 'Position', [500 200 900 700] )
plot( model_nl.time_nl, model_nl.alpha1_nl, '--', model_nl.time_nl, model_nl.alpha2_nl, '--', ...
    model.time, model.alpha1_d, model.time, model.alpha2_d )
title( 'Precession angles' );
legend( 'Precession angle gyro 1, non-linear', 'Precession angle gyro 2, non-linear', ...
    'Precession angle gyro 1, linear', 'Precession angle gyro 2, linear' );
grid on

% ___________________________Precession rate_______________________________
figure( 'Name', 'Precession rates', 'NumberTitle', 'off' )
set( gcf, 'Position', [500 200 900 700] )
plot( model_nl.time_nl, model_nl.alpha1_dot_nl, '--', model_nl.time_nl, model_nl.alpha2_dot_nl, '--', ...
    model.time, model.alpha1_dot,model.time, model.alpha2_dot )
title( 'Precession rates' );
legend( 'Precession rate gyro 1, non-linear', 'Precession rate gyro 2, non-linear', ...
    'Precession rate gyro 1, linear', 'Precession rate gyro 2, linear' );
grid on

% _________________________Precession torque_______________________________
figure( 'Name', 'Precession torque', 'NumberTitle', 'off' )
set( gcf, 'Position', [500 200 900 700] )
plot( model_nl.time_nl, model_nl.torque_nl, '--', model.time, model.torque_d )
title( 'Precession torque' );
legend( 'Precession torque gyro 1, non- linear', 'Precession torque gyro 2, non-linear', ...
    'Precession torque gyro 1, linear', 'Precession torque gyro 2, linear' );
grid on


% ______________________________THE END____________________________________