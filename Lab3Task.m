clear;
clc;
m=1; %mass
k=1; %spring constant
b=1; %damping constant

t=0:0.01:10;

sys=tf(1,[m,b,k]); %transfer function using m,b,k parameters 
%step(sys)
Kp=1;
Ki=1;
Kd=1;

% P control ONLY
C_P=pid(Kp,0,0);
T_P=feedback(sys*C_P,1);
%step(T_P,t)

% PI control ONLY
C_PI=pid(Kp,Ki,0);
T_PI=feedback(sys*C_PI,1);
%step(T_PI,t)

% PD control ONLY
C_PD=pid(Kp,0,Kd);
T_PD=feedback(sys*C_PD,1);
%step(T_PD,t)

% PID control
C_PID=pid(Kp,Ki,Kd);
T_PID=feedback(sys*C_PID,1);
step(T_PID,t)