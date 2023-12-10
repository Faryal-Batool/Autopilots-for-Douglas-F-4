%Study of Lateral Motion
m=39000*0.0312; %mass
g=32.2; % acceleration due to gravity
W=m*g; % Weight
Ix=25000; % Moment of Inertia x-axis 
Iy=122200; %Moment of Inertia y-axis 
Iz=139800;  %Moment of Inertia z·axis
S=530; %Wing Swface
b=38.7; % Wing Span  
AR=(b^2)/S;  % Aspect Ratio
c=16;  %MAC
V=876; % True airspeed
p=0.0007328; % density
Q=283.2;   QS=Q*S; 
M=0.9;
a=QS/(m*V); % dummy variable
e=0.8; % Oswald efficiency number
% Stability derivatives
CL_beta=-0.080;
CL_p=-0.240;
CL_r=0.070;
CY_beta=-0.680;
CY_p=0;
CY_r=0;
Cn_beta=0.125;
Cn_Tbeta=0;
Cn_p=-0.036;
Cn_r=-0.270;
% Control Derivatives
CL_delta_a=0.0420;
CL_delta_R=0.0060;
CY_delta_a=-0.0160;
CY_delta_R=0.095;
Cn_delta_a=-0.0010;
Cn_delta_R=-0.066;

% v derivatives
%Yv
Yv=a*CY_beta;
Y_beta=Yv*V;
%Lv
Lv=((Q*S*b)/(Ix*V))*CL_beta;
L_beta=Lv*V;
%Nv
Nv=((Q*S*b)/(Iz*V))*Cn_beta;
N_beta=Nv*V;


% p derivatives
%Yp
Yp=a*(b/2)*CY_p;
%Lp
Lp=((Q*S*b^2)/(2*Ix*V))*CL_p;
%Np
Np=((Q*S*b^2)/(2*Iz*V))*Cn_p;

% r derivatives
%Yr
Yr=a*(b/2)*CY_r;
%Lr
Lr=((Q*S*b^2)/(2*Ix*V))*CL_r;
%Nr
Nr=((Q*S*b^2)/(2*Iz*V))*Cn_r;

% Control Derivatives
%Delta_a
%L_delta_a
L_delta_a=((Q*S*b)/Ix)*CL_delta_a;
%N_delta_a
N_delta_a=((Q*S*b)/Iz)*Cn_delta_a;

%Delta_r
%L_delta_r
L_delta_r=((Q*S*b)/Ix)*CL_delta_R;
%N_delta_r
N_delta_r=((Q*S*b)/Iz)*Cn_delta_R;
%%Y_delta_r
Y_delta_r=((Q*S)/m)*CY_delta_R;

A1=[Y_beta/V Yp/V -(1-(Yr/V)) g/V;...
   L_beta Lp Lr 0;...
   N_beta Np Nr 0;...
   0 1 0 0];
B1=[0 Y_delta_r/V;...
   L_delta_a L_delta_r;...
   N_delta_a N_delta_r;...
   0 0];
C1=eye(4);
D1=zeros(4,2);
states={'beta','roll','yaw','phi'};
inputs={'aileron','rudder'};
outputs={'Sideslip','roll rate','yaw rate','bank angle'};
System=ss(A1,B1,C1,D1,'statename',states,'inputname',inputs,'outputname',outputs);
damp(A1);
s=tf('s');
H=s/(s+0.02);
%rlocus(-System(3,2)*H)
% % % % % % % % % % % COORDINATED ANGLE HOLD AUTOPILOT % % % % % % %
% % % % % ROLL ANGLE HOLD AUTOPILOT % % % % % % %

% YAW DAMPER
K_DR=0.14;
WOF=H*K_DR;
System1=feedback(System,-WOF,2,3);
damp(System1);

% ROLL DAMPER
K_p=0.9;
System2=feedback(System1,K_p,1,2);

% Roll Attitude feedback to Aileron
sys=System2(4,1);
K_Phi=0.996;
System3=feedback(System2,K_Phi,1,4);

% Sidealip feedback to Rudder
sys2=System3(1,2);
K_beta=5;
System4=feedback(System3,K_beta,2,1);
sys3=System4(4,1);
H1=g/(V*s)*sys3;
K_psi=1;
H2=feedback(H1,K_psi,-1)
figure()
step(H2)


