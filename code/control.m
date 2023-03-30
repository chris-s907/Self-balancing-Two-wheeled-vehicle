clc
clear
set(0,'defaultfigurecolor','w')
%% initial condition
a = 3;
b = 2;
c = 5;
d = 2;

%parameters
Mf = 2.14 + c/20; %mass of front(kg)
Mr = 5.91 - b/10; %mass of body(rear part)(kg)
Mc = 1.74; %mass of rider's center-of-gravity (kg)
Hf = 0.18; %vertical length from floor to front part(m)
Hr = 0.161; %vertical length from floor to body(m)
Hc = 0.098; %vertical length from floor to rider's center-of-gravity(m)
LFf = 0.05; %Horizontal length from a front wheel rotation axis to a center-of-gravity of part of front wheel
LF = 0.133; %Horizontal length from a front wheel rotation axis to a center-of-gravity of part of steering axis
Lr = 0.128; %Horizontal length from a rear wheel rotation axis to a center-of-gravity of part of rear wheel
LR = 0.308 + (a - d)/100; %Horizontal length from a rear wheel rotation axis to a center-of-gravity of part of steering axis
Lc = 0.259; %Horizontal length from a rear wheel rotation axis to a center-of-gravity of the cart system
Jx = 0.5 + (c - d)/100; %Moment of inertia around center-of-gravity x axially
ux = 3.33 - b/20 + a * c/60; %Viscous coefficient around x axis

alpha = 15.5 - a/3 + b/2;
beta = 27.5 - d/2;
gama = 11.5 + (a - c)/(b + d + 3);
delta = 60 + (a - b)*c/10;
g = 9.8;

%coefficient of matrix
den = Mf * Hf^2 + Mr * Hr^2 + Mc * Hc^2 + Jx;
a_51 = -(Mc * g)/den;
a_52 = (Mf * Hf + Mr * Hr + Mc * Hc)*g/den;
a_53 = (Mr * Lr * LF + Mc * Lc * LF + Mf * LFf * LR)*g/((LR + LF)*den);
a_54 = -(Mc * Hc * alpha)/den;
a_55 = - ux/den;
a_56 = Mf * Hf * LFf * gama/den;
b_51 = Mc * Hc * beta/den;
b_52 = - Mf * Hf * LFf * delta/den;

%matrix A, B, C 
A = [0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     0, 6.5, -10, -alpha, 0, 0;
     a_51, a_52, a_53, a_54, a_55, a_56;
     5, -3.6, 0, 0, 0, -gama];
 
 B = [0, 0;
      0, 0;
      0, 0;
      beta, 11.2;
      b_51, b_52;
      40, delta];
  
 C = [1, 0, 0, 0, 0, 0;
      0, 1, 0, 0, 0, 0;
      0, 0, 1, 0, 0, 0];
  
 D = zeros(3,2);
 
states = {'d' 'phi' 'psi' 'd_dot' 'phi_dot' 'psi_dot'};
inputs = {'u_c' 'u_h'};
outputs = {'car postition'; 'handle angle'; 'bike angle'};
outputs2 = {'car postition';  'bike angle'};
 
 % initial state
 x0 = [0.2; -0.1; 0.15; -1; 0.8; 0];
 t = 0:0.01:10;
 
%% state space to transform function
 sf = ss(A, B, C, D,'statename',states,'inputname',inputs,'outputname',outputs);
 G = tf(sf);
%  step(sf ,t)
%  title('Step response of Open-Loop system')


%  lsim(sf,zeros(2,1001),t,x0)
%  title('response of non-zero initial state with zero initial inputs')


%% check the controbaility
 con_matrix = ctrb(A, B);
 ro = rank(con_matrix);
 
 
%% pole set step
Co = zeros(size(A));
Co(:,1) = con_matrix(:,1);
Co(:,2) = con_matrix(:,3);
Co(:,3) = con_matrix(:,5);
Co(:,4) = con_matrix(:,2);
Co(:,5) = con_matrix(:,4);
Co(:,6) = con_matrix(:,6);
Co_inv  = inv(Co);

T(1,:) = Co_inv(3,:);
T(2,:) = Co_inv(3,:) * A;
T(3,:) = Co_inv(3,:) * A * A;
T(4,:) = Co_inv(6,:);
T(5,:) = Co_inv(6,:) * A;
T(6,:) = Co_inv(6,:) * A * A;
T_inv = inv(T);

format short
A_bar = T * A * T_inv;

format short
B_bar = T * B;

for i = 1:6
    for j = 1:6
        if(abs(A_bar(i,j)) < 0.001) A_bar(i,j) = 0;
        end
    end
end

for i = 1:6
    for j = 1:2
        if(abs(B_bar(i,j)) < 0.001) B_bar(i,j) = 0;
        end
    end
end

 %% pole set
%  syms s
 P1 = [-1+0.75i, -1-0.75i, -2, -2, -4, -4];%ideal poles
 P2 = [-0.8+0.6i, -0.8-0.6i, -1.5, -1.5, -3, -3];
 P3 = [-2+1.5i, -2-1.5i, -4, -4, -10, -10];
 K1 = place(A, B, P1);
%  K_bar = K1 * T_inv
%  A_d = A_bar - B_bar * K_bar
 K2 = place(A, B, P2);
 K3 = place(A, B, P3);
 sys_cl = ss(A-B*K1,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
 sys_c12 = ss(A-B*K2,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
 sys_c13 = ss(A-B*K3,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

% lsim(sys_c12,zeros(2,1001),t,x0)
% hold on
% lsim(sys_cl,zeros(2,1001),t,x0)
% hold on
% lsim(sys_c13,zeros(2,1001),t,x0)

%  step(sys_c12, t)
%  hold on
%  step(sys_cl, t);
%  hold on
%  step(sys_c13, t)
%  legend('dominant poles with -0.8','dominant poles with -1','dominant poles with -2')

 
%% LQR
 Q = diag([5 10 1 0 0 0]);
  
 Q2 = diag([30 10 1 0 0 0]);
%  Q2 = diag([5 30 1 0 0 0]);
%  Q2 = diag([5 10 10 0 0 0]);

 Q3 = diag([100 10 1 0 0 0]);
%  Q3 = diag([5 100 1 0 0 0]);
% Q3 = diag([5 10 100 0 0 0]);

R = [0.5, 0;
      0, 0.5];
  
% R2 = [1, 0;0, 0.5];
% R2 = [0.5, 0;0, 1];
  
% R3 = [3, 0;0, 0.5];
% R3 = [0.5, 0;0, 3];

%LQR step
Tao_11 = A;
Tao_12 = (-1) * B * inv(R) * B';
Tao_21 = (-1) * Q;
Tao_22 = (-1) * A';
Tao =[Tao_11, Tao_12;
      Tao_21, Tao_22];
format short
[e_vector,e_value] = eig(Tao);

 K2 = lqr(A, B, Q, R);
 K22 = lqr(A, B, Q2, R);
% K22 = lqr(A, B, Q, R2);
 K23 = lqr(A, B, Q3, R);
% K23 = lqr(A, B, Q, R3);
 sys_c2 = ss(A-B*K2,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
 sys_c22 = ss(A-B*K22,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
 sys_c23 = ss(A-B*K23,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

% step(sys_c2, t);

% lsim(sys_c2,zeros(2,1001),t,x0)
% step(sys_c2, t)
% hold on
% lsim(sys_c22,zeros(2,1001),t,x0)
% step(sys_c22, t)
% hold on
% lsim(sys_c23,zeros(2,1001),t,x0)
% step(sys_c23, t)

% title('response of non-zero initial state with zero initial inputs')


%  legend('Q(1,1)=5','Q(1,1)=30','Q(1,1)=100')
%  legend('Q(2,2)=10','Q(2,2)=30','Q(2,2)=100')
%  legend('Q(3,3)=1','Q(3,3)=10','Q(3,3)=100')
%  legend('Q(4,4)=0','Q(4,4)=10','Q(4,4)=100')
%  legend('Q(5,5)=0','Q(5,5)=10','Q(5,5)=30')
%  legend('Q(6,6)=0','Q(6,6)=10','Q(6,6)=30')
%  legend('R(1,1)=0.5','R(1,1)=1','R(1,1)=3')
%  legend('R(2,2)=0.5','R(2,2)=1','R(2,2)=3')
 
 
%% Servo control
% y_sp =0.1 * C * inv(A)* B * [-0.5+(a-b)/20; 0.1+(b-c)/(a+d+10)];
y_sp = -0.2 * C * inv(A)* B *[0.2;0.1];
% y_sp = [2;pi/4;pi/4];
% y_sp = [-2;-pi/4;-pi/4];

%controllability check
con_matrix2 = [A, B; -C, zeros(3,2)];
ro2 = rank(con_matrix2);

%% Decouple system
syms s 
s_I = s*eye(6);
C2 = [1, 0, 0, 0, 0, 0;
      0, 0, 1, 0, 0, 0];
c1_TAB = [1, 0, 0, 0, 0, 0] *A*B;
c2_TAB = [0, 0, 1, 0, 0, 0] *A*B;
c1_TA2 = [1, 0, 0, 0, 0, 0] *(A*A + 2*A + eye(6));
c2_TA2 = [0, 0, 1, 0, 0, 0] *(A*A + 2*A + eye(6));
B_star = [c1_TAB;c2_TAB];
C_star = [c1_TA2;c2_TA2];
K4 = inv(B_star) * C_star;
F = inv(B_star);
% A_prim = A - B*K4;
% B_prim = B * F;
% simplify the coefficient of matrix B_prim and A_prim
B_prim = [0, 0;
     0, 0;
     0, 0;
     1, 0;
     0.36, 0;
     0, 1];

A_prim = [0, 0, 0, 1, 0, 0;
        0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 1;
        -1, 0, 0, -2, 0, 0;
        -22, 16.6, 9.2, 1.5, -4.5, -0.6;
        0, 0, -1, 0, 0, -2];
H = C2 * inv(s_I - A_prim)* B_prim;

sys4 = ss(A_prim, B_prim, C2, zeros(2,2),'statename',states,'inputname',inputs,'outputname',outputs2);
% step(sys4, t);
% lsim(sys4,zeros(2,1001),t,x0)

%% observer design
%check the observability
Vo = obsv(A,C);
P_obse = [-3+0.75i, -3-0.75i, -6, -6, -12, -12];%ideal
% P_obse = [-1+0.75i, -1-0.75i, -2, -2, -4, -4];
% P_obse = [-5+0.75i, -5-0.75i, -10, -10, -20, -20];
% P_obse = [-10+0.75i, -10-0.75i, -50, -50, -80, -80];
Lt = place(A',C',P_obse);
L = Lt';

% evalin('base','sim(''que3.slx'')')
% 
% subplot(2,3,1)
% plot(x.time,x.data(:,1),x_hat.time, x_hat.data(:,1),x.time,x.data(:,1)-x_hat.data(:,1))
% legend('car position','car position estimation','car position error')
% ylabel('car position£¨m£©');
% xlabel('Time(s)');
% grid on
% 
% subplot(2,3,2)
% plot(x.time,x.data(:,2),x_hat.time, x_hat.data(:,2),x.time,x.data(:,2)-x_hat.data(:,2))
% legend('handle angle','handle angle estimation','handle angle error')
% ylabel('handle angle£¨rad£©');
% xlabel('Time(s)');
% grid on
% 
% subplot(2,3,3)
% plot(x.time,x.data(:,3),x_hat.time, x_hat.data(:,3),x.time,x.data(:,3)-x_hat.data(:,3))
% legend('bike angle','bike angle estimation','bike angle error')    
% ylabel('bike angle£¨rad£©');
% xlabel('Time(s)');
% grid on
% 
% subplot(2,3,4)
% plot(x.time,x.data(:,4),x_hat.time, x_hat.data(:,4),x.time,x.data(:,4)-x_hat.data(:,4))
% legend('car velocity','car velocity estimation','car velocity error')
% ylabel('car velocity£¨m/s£©');
% xlabel('Time(s)');
% grid on

% subplot(2,3,5)
% plot(x.time,x.data(:,5),x_hat.time, x_hat.data(:,5),x.time,x.data(:,5)-x_hat.data(:,5))
% legend('handle angular velocity','handle angular velocity estimation','handle angular velocity error')
% ylabel('handle angular velocity£¨rad/s£©');
% xlabel('Time(s)');
% grid on
% 
% subplot(2,3,6)
% plot(x.time,x.data(:,6),x_hat.time, x_hat.data(:,6),x.time,x.data(:,6)-x_hat.data(:,6))
% legend('bike angular velocity','bike angular velocity estimation','bike angular velocity error')
% ylabel('bike angular velocity£¨rad/s£©');
% xlabel('Time(s)');
% grid on

%% Setpoint tracking
%stat space model
A_5 = [A,zeros(6,3);
       -C, zeros(3,3)];
B_5 = [B;zeros(3,2)];
C_5 = [C,zeros(3,3)];
D_5 = zeros(3,2);
Bw_5 = [B;zeros(3,2)];
Br_5 = [zeros(6,3);eye(3)];
Q_5 = eye(9);
Q_5(1,1) =5;
Q_5(2,2) = 10;
R_5 = eye(2);
K5 = lqr(A_5, B_5, Q_5, R_5);
% t2 = 0:0.01:30;
% x02 = [0.2; -0.1; 0.15; -1; 0.8; 0;0;0;0];
% sys_c5 = ss(A_5-B_5*K5,B_5,C_5,D_5,'inputname',inputs,'outputname',outputs);
% step(sys_c5, t2);
% lsim(sys_c5,zeros(2,3001),t2,x02)

for i=1:6
    K5_obse(:,i) = K5(:,i);
end

evalin('base','sim(''qus5.slx'')')
subplot(1,3,1)
plot(ref.time,ref.data(:,1),'--',output.time,output.data(:,1),output.time,output.data(:,1)-y_sp(1,1),'.')
legend('set point','car position','error')
axis([0 30 -5 5])
grid on
ylabel('car position£¨m£©');
xlabel('Time(s)');

subplot(1,3,2)
plot(ref.time,ref.data(:,2),'--',output.time,output.data(:,2),output.time,output.data(:,2)-y_sp(2,1),'.')
legend('set point','handle angle','error')
axis([0 30 -5 5])
grid on
ylabel('handle angle£¨rad£©');
xlabel('Time(s)');

subplot(1,3,3)
plot(ref.time,ref.data(:,3),'--',output.time,output.data(:,3),output.time,output.data(:,3)-y_sp(3,1),'.') 
legend('set point','bike angle','error')
axis([0 30 -5 5])
grid on
ylabel('bike angle£¨rad£©');
xlabel('Time(s)');
%      
% plot(control.time,control.data(:,1),control.time,control.data(:,2)) 
% legend('input u_c','input u_h')
% xlabel('Time(s)');
% grid on
 

