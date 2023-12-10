%Study of Longitudianl Motion
m=39000*0.0311; %mass
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
Q=283.2;   QS=Q*S; 
M=0.9;
a=QS/(m*V); % dummy variable
e=0.8;
% Steady state
CL=0.26;
CD=0.030;
Cm=0;
CTx=0.030;
CmT=0;
% Stability derivatives
CTu=0; % because this is a jet aircraft
CD_0=0.0205;
CDu=0.027;
CDalpha=0.30;
CL_0=0.10;
CLu=0.270;
CLalpha=3.75;
CLalphadot=0.86;
Cmu=-0.117;
Cma=-0.40; 
Cmadot=-1.30;
Cmq=-2.70;
% Control derivatives % just an assumption: ignore throttle settings
CXdelta_e=-0.10;
CZdelta_e=0.40;
Cmdelta_e=-0.580;


%The u derivatives
%Xu
Xu=(a)*(-(CDu+(2*CD)))+CTu;
%Zu
Zu=-(a)*(2*CL+CLu);
%Mu
Mu=((QS*c)/(Iy*V))*Cmu;

%The w derivatives
%Xw
s=(2*CL)/(pi*e*AR); % dummy variable
Xw=a*(CL-(s*CLalpha));
Xa=Xw*V;
%Zw
Zw=(a)*(-CLalpha-CD);
Za=V*Zw;
%Mw
Mw=((QS*c)/(Iy*V))*Cma;
Ma=V*Mw;
%Mwdot
Mwdot=((QS*c^2)/(2*(Iy)*(V^2)))*Cmadot;
Madot=V*Mwdot;

%The q derivatives
%Mq
 Mq=((QS*c^2)/(2*Iy*V))*Cmq;

%The delta e derivatives
%Xdelta_e
Xdelta_e=(QS/m)*CXdelta_e;

%Zdelta_e
Zdelta_e=(QS/m)*CZdelta_e;

%Mdelta_e
Mdelta_e=((QS*c^2)/Iy)*Cmdelta_e;

A=[Xu Xa 0 -g; Zu/V Za/V 1 0; Mu+(Madot*Zu/V) Ma+(Madot*Za/V) Mq+Madot 0; 0 0 1 0];
B=[Xdelta_e Zdelta_e/V Mdelta_e+(Madot*Zdelta_e/V) 0]';
C=[0 0 0 1];
D=[0]';
System=ss(A,B,C,D);
%damp(A);

% SAS DESIGN
% Pitch Attitude feedback
K_theta=0.6;
sys1=feedback(System*K_theta,1,+1);
[A2,B2,C2,D2]=ssdata(sys1);
C3=[0 0 1 0];
sys2=ss(A2,B2,C3,D2);

% Ptch Rate feedback
K_q=0.4; % Value is changed from 0.0155 to K_q=0.3 to reduce oscillations in response
sys3=feedback(sys2,K_q,+1);
figure()
step(sys3)
[A3,B3,C3,D3]=ssdata(sys3);
C4=[0 0 0 1];
sys4=ss(A3,B3,C4,D3);
figure()
step(-sys4)
stepinfo(sys4)
KP=-5;
C=pid(KP,0,0);
I=feedback(C*sys4,1);
figure()
step(I)
stepinfo(I)
damp(I)
%controlSystemDesigner('rlocus',-sys4)
controlSystemDesigner('pitch autopilot')


%% Height
%Study of Longitudianl Motion
m=39000*0.0311; %mass
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
Q=283.2;   QS=Q*S; 
M=0.9;
a=QS/(m*V); % dummy variable
e=0.8;
% Steady state
CL=0.26;
CD=0.030;
Cm=0;
CTx=0.030;
CmT=0;
% Stability derivatives
CTu=0; % because this is a jet aircraft
CD_0=0.0205;
CDu=0.027;
CDalpha=0.30;
CL_0=0.10;
CLu=0.270;
CLalpha=3.75;
CLalphadot=0.86;
Cmu=-0.117;
Cma=-0.40; 
Cmadot=-1.30;
Cmq=-2.70;
% Control derivatives % just an assumption: ignore throttle settings
CXdelta_e=-0.10;
CZdelta_e=0.40;
Cmdelta_e=-0.580;


%The u derivatives
%Xu
Xu=(a)*(-(CDu+(2*CD)))+CTu;
%Zu
Zu=-(a)*(2*CL+CLu);
%Mu
Mu=((QS*c)/(Iy*V))*Cmu;

%The w derivatives
%Xw
s=(2*CL)/(pi*e*AR); % dummy variable
Xw=a*(CL-(s*CLalpha));
Xa=Xw*V;
%Zw
Zw=(a)*(-CLalpha-CD);
Za=V*Zw;
%Mw
Mw=((QS*c)/(Iy*V))*Cma;
Ma=V*Mw;
%Mwdot
Mwdot=((QS*c^2)/(2*(Iy)*(V^2)))*Cmadot;
Madot=V*Mwdot;

%The q derivatives
%Mq
 Mq=((QS*c^2)/(2*Iy*V))*Cmq;

%The delta e derivatives
%Xdelta_e
Xdelta_e=(QS/m)*CXdelta_e;

%Zdelta_e
Zdelta_e=(QS/m)*CZdelta_e;

%Mdelta_e
Mdelta_e=((QS*c^2)/Iy)*Cmdelta_e;

A=[Xu Xa 0 -g; Zu/V Za/V 1 0; Mu+(Madot*Zu/V) Ma+(Madot*Za/V) Mq+Madot 0; 0 0 1 0];
B=[Xdelta_e Zdelta_e/V Mdelta_e+(Madot*Zdelta_e/V) 0]';
C1=[0 0 1 0];
D1=[0]';
System=ss(A,B,C1,D1);


K_q=0.4;
sys1=feedback(System,K_q,+1);
[A2,B2,C2,D2]=ssdata(sys1);
C3=[0 0 0 1];
sys2=ss(A2,B2,C3,D2);


K_theta=0.6;
sys3=feedback(sys2,K_theta,+1);
[A3,B3,C3,D3]=ssdata(sys3);
C4=[0 1 0 0;0 0 0 1];
D4=[0 0]';
sys4=ss(A3,B3,C4,D4);
I=tf(sys4);
s=tf('s');
G=tf((V/s)*(I(2)-I(1)));
figure()
step(G)
stepinfo(G)
figure()
rlocus(-G)
KP=0.000289;
C=pid(KP,0,0)
height=feedback(C*G,1,+1)
figure()
step(-height)
stepinfo(height)

