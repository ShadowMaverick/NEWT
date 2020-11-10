%   Implementing a LQR controller for a two-gyroscope system.
%   The equations as the elements of the matrices is taken from
%   Ioannis report. Some modifications will be made.
%   
%   Author:     NEWT team
%   Course:     MF2059 - Mechatronic higher course
%   Date:       2019-09-08
%   Coach:      Lasse Hässler/ Didem
%   University: KTH - Royal Institure of Technology

clear all, close all, clc

%   Parameters  --  Parameters --   Parameters --   Parameters -- %
omega   = 532.6;                 % [rad/s]
%omega=0;
g       = 9.81;                 % [m/s^2]
% Gyro - flywheel inertia
Igz     = 0.061;                % [kg*m^2]
Igx     = Igz/2;                % [kg*m^2]
Igy     = Igz/2;                % [kg*m^2]
% Gyrp - cage inertia
Icx     = 0.04;                 % [kg*m^2]
Icy     = 0.06;                 % [kg*m^2]
Icz     = 0.1;                  % [kg*m^2]
% Gyro-base inertia
Igbx    = 0;                % [kg*m^2]
% Vehicle inertia
Ibx     = 10.2;                 % [kg*m^2]
% Servo inertia
Isx     = 0;                % [kg*m^2]
% Mass of flywheel
mg      = 8;                    % [kg]
% Mass of cage
mc      = 5;                    % [kg]
% Gyro-base mass & height of COG
mgb     = 0;                    % [kg]
hg      = 0.37;                 % [m]
% Servo mass & height of COG
ms      = 0;                    % [kg]
hs      = 0;                    % [m]
% Vehicle mass & height of COG
mb      = 45;                   % [kg]
hb      = 0.37;                 % [m]
% Circuit properties
gamma   = 1;
Kt      = 48/6;
Ke      = Kt;
%Ke = 48/4.189;                 % [V/(rads/s)], V=48, rpm=40=4.189rad/s, Ke=Kt
Km      = 1;                    % [N*m/sqrt(W)]
L       = 1;                    % Henry
R       = 1;                    % Ohm


%   Equations -- Equations --   Equations --    Equations -- %
k1 = 2*( Icx + Igx );
k2 = 2*( Icy + Igy );
k3 = 2*( Icz + Igz );
k4 = Ibx + 2*Igbx + 2*Isx;
k5 = k1 - k3;
k6 = hb*mb + 2*hg*(mg + mc + mgb) + 2*hs*ms;
k7 = ( hb^2 )*mb + 2*( hg^2 )*( mg + mc + mgb ) + 2*( hs^2 )*ms;

%   Matrices --     Matrices --     Matrices --     Matrices -- %
% States are x = [theta, alpha, theta_dot, alpha_dot]
a31 = ( g*k6 )/( k1 + k7 + k4 );
a34 = -( 2*omega*Igz )/( k1 + k7 + k4 );
a43 = ( 2*omega*Igz )/k2;
a44 = -gamma/k2;
a45 = Ke/k2;
a54 = -Km/L;
a55 = -R/L;

A = [0, 0, 1, 0
    0, 0, 0, 1
    a31, 0, 0, a34
    0, 0, a43, 0];

b41 = 1/k2;

B = [0; 0; 0; b41];

C = [1, 0, 0, 0
    0, 1, 0, 0
    0, 0, 1, 0
    0, 0, 0, 1];

D = [0; 0; 0; 0];

% Controllability + Observability --- Controllability + Obesrvability --- %
Controllability = ctrb( A, B );                   % Controllability matrix
r_c = rank( Controllability );                      % Check the rank of controllability matrix

O = obsv( A, C );
r_o = rank( O );

% Tune Q and R + finding K-value --- Tune Q and R + finding K-value --- %
% Q = [20000, 0, 0, 0     % Theta
%     0, 9000, 0, 0      % Alpha
%     0, 0, 345, 0      % Theta_dot
%     0, 0, 0, 234];    % Alpha_dot     
% 
% R = 1;

Q = [3500, 0, 0, 0     % Theta
    0, 2789, 0, 0      % Alpha
    0, 0, 345, 0      % Theta_dot
    0, 0, 0, 234];    % Alpha_dot     

R = 1;

K = lqr( A, B, Q, R );

sys = ss(( A - B*K), B, C, D );
t = 0:0.1:40;
x0 = [10, 0, 0, 0];
[ y, T, x ] = initial( sys, x0, t );
%plot(T, y(:,1));
%figure()
%plot(T, y(:,2));
%figure()
%plot(T, -K*x');

% Initial conditions ---- Initial conditions ---- Initial conditions ---- %
roll_angel          = 0.34;     % [rad]
precession_angle    = 0;        % [rad]
roll_rate           = 0;        % [rad/s]
precession_rate     = 0;        % [rad/s]

Init_cond = [roll_angel, precession_angle, roll_rate, precession_rate];


init_roll_angle = 0.34;
init_precession_angle = 0;
init_roll_rate = 0;
init_precession_rate = 0;
F_d = 0;


sim('LQR_nonlinearized')
figure()
plot(time, rad2deg(Alpha), time, rad2deg(LinAlpha))
title('Precession angle');
legend('Non-linearized','Linearized');
grid on

figure()
plot(time, rad2deg(Theta), time, rad2deg(LinTheta))
title('Roll angle');
legend('Non-linearized','Linearized');
grid on

figure()
plot(time, rad2deg(Alphadot), time, rad2deg(LinAlphadot))
title('Precession rate');
legend('Non-linearized','Linearized');
grid on

figure()
plot(time, rad2deg(Thetadot), time, rad2deg(LinThetadot))
title('Roll rate');
legend('Non-linearized','Linearized');
grid on

figure()
plot(time, Torque, time, LinTorque)
title('Torque');
legend('Non-linearized','Linearized');
grid on






