%Parámetros
l=0.413;        %Longitud pendulo
M=0.01;         %Masa brazo
Jp=0.0009;      %Momento inercia brazo
r=0.235;        %Longitud brazo
J=0.05;         %Momento inercia pendulo
m=0.02;         %Masa pendulo
g=9.807;        %Gravedad

a=Jp + M*l^2;
b=J + M*r^2 + m*r^2;
c=M*r*l;
d=l*g*(M+m/2);

%Espacio de estados
A=[0 1 0 0; (b*d)/(a*b-c^2) 0 0 0; 0 0 0 1; -(c*d)/(a*b-c^2) 0 0 0];
B=[0; -c/(a*b-c^2); 0; a/(a*b-c^2)];
C=eye(4);
D=zeros(4,1);

sys=ss(A,B,C,D);
eig(A);         %Polos en lazo abierto

%Ganancia original de la guía
K=[-7.5343 -1.3465 0 -0.2216];
A_re=A-B*K;     %Matriz A realimentada

sys_re=ss(A_re,B,C,D);
eig(A_re);       %Polos con realimentacion

%Controlador LQR
Q=diag([10 1 10 0.1]);
R=[1000];

[K_lqr, S_lqr, eig_lqr] = lqr(A, B, Q, R);
A_lqr=A-B*K_lqr;

sys_lqr=ss(A_lqr,B,C,D);
eig_lqr;       %Polos con LQR

%Variables para gráficas
x_0=[5*pi/180; 0; 0; 0];  %Posición inicial: 5 grados en theta
t=0:0.01:10;
[y,t,x]=initial(sys_lqr, x_0, t);
u=-x*K_lqr';

%Gráficas del comportamiento del péndulo
figure
subplot(2,1,1)
plot(t, x(:,1))
ylabel('\theta (rad)')
title('Péndulo')
grid on

subplot(2,1,2)
plot(t, x(:,2))
ylabel('d\theta/dt (rad/s)')
xlabel('Tiempo (s)')
grid on

%Gráficas del comportamiento del brazo
figure
subplot(2,1,1)
plot(t, x(:,3))
ylabel('\phi (rad)')
title('Brazo')
grid on

subplot(2,1,2)
plot(t, x(:,4))
ylabel('d\phi/dt (rad/s)')
xlabel('Tiempo (s)')
grid on

%Gráficas del esfuerzo del controlador
figure
plot(t,u)
title('Respuesta del controlador')
ylabel('u(t)')
xlabel('Tiempo (s)')
grid on

%Discretización
Ts=0.002;
sys_d=c2d(sys,Ts,'tustin');
%sys_d=c2d(sys,Ts,'zoh');
A_d=sys_d.A;
B_d=sys_d.B;

[K_d_lqr, S_d_lqr, eig_d_lqr]=dlqr(A_d,B_d,Q,R);
A_d_lqr=A_d-B_d*K_d_lqr;

sys_d_lqr=ss(A_d_lqr,B_d,C,D);
eig_d_lqr;       %Polos con LQR

[y_d, t_d, x_d]=initial(sys_d_lqr,x_0,t);
u_d=-x_d*K_d_lqr';