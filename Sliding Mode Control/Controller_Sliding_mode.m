clear all;
clc;

%% Simulation Parameters
Ts = 0.02;         % Sampling time [s]
time = 20;         % Total simulation time [s]

% Sliding mode controller parameters
alpha = 6;         % Gain for angle error feedback
beta = 1.8;        % Gain for position error feedback
var_k = 10;        % Sliding mode gain

%% State Initialization
% q(1): cart position, q(2): pendulum angle
q   = zeros(2,time/Ts);
qd  = zeros(2,time/Ts); % velocities
qdd = zeros(2,time/Ts); % accelerations

% Initial conditions
q(:,1) = [0.4; -0.1]; 

% Control input
tau = zeros(1,time/Ts);

% References (not directly used here)
q_r  = [0; 0];
qd_r = zeros(2,1);

%% Physical Parameters
m_p = 0.329;  % Pendulum mass [kg]
m_w = 3.2;    % Cart mass [kg]
l_sp = 0.44;  % Pendulum length [m]
f_w = 6.2;    % Cart friction
f_p = 0.009;  % Pendulum friction
gra = 9.81;   % Gravity [m/s^2]
j_a = 0.072;  % Pendulum inertia

%% Continuous-time state-space model
A_c = [ 0   1                               0                   0
        0   -f_w/(m_w+m_p)                  0                   0
        0   0                               0                   1
        0   (f_w*m_p*l_sp)/(j_a*(m_w+m_p)) (m_p*l_sp*gra)/j_a     -f_p/j_a];   
B_c = [0; 1/(m_w+m_p); 0; -m_p*l_sp/((m_w+m_p)*j_a)];
C_c = eye(4);
D_c = zeros(4,1);

% Discretize model (only for completeness)
sys_cont = ss(A_c,B_c,C_c,D_c);
sys_d = c2d(sys_cont,Ts);
A = sys_d.A; B = sys_d.B;

%% Nonlinear pendulum parameters
KF=2.6; M0=3.2; M1=0.329; M=M0+M1; ls=0.44; inert=0.072;
N_val=0.1446; N01_sq=0.23315; Fr=6.2; C=0.009; gra=9.81;

a32 = -N_val^2/N01_sq*gra; 
a33 = -inert*Fr/N01_sq; 
a34 = N_val*C/N01_sq; 
a35 = inert*N_val/N01_sq; 
a42 = M*N_val*gra/N01_sq; 
a43 = N_val*Fr/N01_sq; 
a44 = -M*C/N01_sq;
a45 = -N_val^2/N01_sq; 
b3  = inert/N01_sq; 
b4  = -N_val/N01_sq;

%% Animation setup
xmin = -1; xmax = +1;
figure;
h = [];
h(1) = subplot(4,2,1); % Position
h(2) = subplot(4,2,3); % Angle
h(3) = subplot(4,2,5); % Velocity
h(4) = subplot(4,2,7); % Angular velocity
h(5) = subplot(2,2,2); % Torque
h(6) = subplot(2,2,4); % Cart-pendulum animation

%% Control and simulation loop
for k = 1:time/Ts-1

    % --- Sliding mode-like control law ---
    s = - qd(2,k) - alpha*q(2,k) - qd(1,k) - beta*q(1,k); 
    tau(1,k) = 12.335239*qd(1,k) + 68.877358*q(2,k) ...
               - 3.492138*(0.125-alpha)*qd(2,k) ...
               - 3.492138*(1.7568-beta)*qd(1,k) ...
               - (var_k * sign(s));

    % Saturate control input
    if abs(tau(:,k)) > 10
        tau(:,k) = sign(tau(:,k)) * 10;
    end

    % --- Nonlinear dynamics ---
    beta_x2 = (1+N_val^2/N01_sq*(sin(q(2,k)))^2)^(-1);

    qdd(:,k+1) = [ ...
        beta_x2*(a32*sin(q(2,k))*cos(q(2,k)) + a33*qd(1,k) + ...
                 a34*cos(q(2,k))*qd(2,k) + a35*sin(q(2,k))*qd(2,k)^2 + ...
                 b3*tau(:,k));
        beta_x2*(a42*sin(q(2,k)) + a43*cos(q(2,k))*qd(1,k) + ...
                 a44*qd(2,k) + a45*cos(q(2,k))*sin(q(2,k))*qd(2,k)^2 + ...
                 b4*cos(q(2,k))*tau(:,k))];

    % Integrate states
    qd(:,k+1) = qd(:,k) + qdd(:,k+1)*Ts;        
    q(:,k+1) = q(:,k) + qd(:,k+1)*Ts;
    q(2,k+1) = mod(q(2,k+1)+pi,2*pi)-pi; % Wrap angle between -pi and pi

    % --- Animation ---
    plot(0,'Parent',h(6));
    hold on;
    p1 = -q(1,k);
    p2 = -q(1,k) + ls*exp(1i*(q(2,k)+pi/2));
    line(real([p1,p2]), imag([p1,p2]));
    plot(real(p2), imag(p2), '.', 'markersize', 40);
    hold off;

    % Adjust animation window
    if q(1) > xmax
        xmin = xmin + 0.1; xmax = xmax + 0.1;
    elseif q(1) < xmin
        xmin = xmin - 0.1; xmax = xmax - 0.1;
    end

    % Axes settings
    axis([h(1)],[0 time/Ts-1 -1 1]);
    axis([h(2)],[0 time/Ts-1 -0.5 0.5]);
    axis([h(3)],[0 time/Ts-1 -1 1]);
    axis([h(4)],[0 time/Ts-1 -5 5]);
    axis([h(5)],[0 time/Ts-1 -10 10]);
    axis([h(6)],[xmin-.2 xmax+.2 -.5 .5]);
    grid on;
    drawnow;

    % --- Update plots ---
    plot(q(1,1:k),'b','Parent',h(1)); % Position
    plot(q(2,1:k),'b','Parent',h(2)); % Angle
    plot(qd(1,1:k),'b','Parent',h(3)); % Velocity
    plot(qdd(1,1:k),'b','Parent',h(4)); % Acceleration
    plot(tau(1:k),'k','Parent',h(5));   % Control input

    % Titles
    title(h(1), 'Position');
    title(h(2), 'Angle');
    title(h(3), 'Velocity');
    title(h(4), 'Angular velocity');
    title(h(5), 'Torque');

end