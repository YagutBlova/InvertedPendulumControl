%%
pause(3);
clear all;
close all;
clc;

addpath(genpath('CLSS Praxis'),genpath('hudaqlib'))
dev = HudaqDevice('MF634');
mn = 25;
mkdir Messung_25;


s = 2000;  %total experiment sample
Ts = 0.02; % sampling period


x = zeros(s,4); %states during experiment
z2 = zeros(4,1);% sample states
x(1,:) = [(AIRead(dev,1)/0.15/100) 0.01*round(-13.1*AIRead(dev,3)) (-AIRead(dev,2)/0.96*pi/180) 0]; % inital values
Force = zeros(s,1); 

Info= strings([s,1]);
q_r_position_angle = [0 0];  % reference value of position
q_r_angle = 0;  % reference value of angle

%%%%%%%%% your part

%% State-space
%%
m_p     = 0.329;m_w     = 3.2;l_sp    = 0.44;f_w     = 6.2; 
f_p     = 0.009;gra       = 9.81;j_a     = 0.072;Ts = 0.02; 

A_c = [ 0   1                               0                   0
        0   -f_w/(m_w+m_p)                  0                   0
        0   0                               0                   1
        0   (f_w*m_p*l_sp)/(j_a*(m_w+m_p)) (m_p*l_sp*gra)/j_a     -f_p/j_a];   
B_c = [0  ;   1/(m_w+m_p) ;   0   ;   -m_p*l_sp/((m_w+m_p)*j_a)];
C_c = [   1   0   0   0
        0   1   0   0
        0   0   1   0
        0   0   0   1];
D_c = [0;0;0;0];

sys_cont = ss(A_c,B_c,C_c,D_c);
sys_d = c2d(sys_cont,Ts);

A = sys_d.A;B = sys_d.B;C = sys_d.C;D = sys_d.D;

%%

%%%%%%%%%%%%%%%%%%%%%
%fuzzy_angle_matlab = readfis('fuzzy_angle_position_simple_v64-Side_B_2020a');
fuzzy_angle_position_matlab = readfis('fuzzy_angle_position_simple_v169-Side_B_2020a2');


for i = 1:s-1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  CONTROL ALGORITHM

q_r_position = x(i,1);  % reference value of position
q_r_car_speed = x(i,2);
q_r_angle = x(i,3);  % reference value of angle
q_r_angle_velocity = z2(4);

if q_r_angle<-0.2
    q_r_angle=-0.2;
end
if q_r_angle>0.2
    q_r_angle=0.2;
end
%Angle
%evaluate_fuzzy = evalfis(fuzzy_angle_matlab,q_r_angle);

%Angle_and_position
q_r_position_angle=[q_r_angle,q_r_angle_velocity,q_r_position,q_r_car_speed];
evaluate_fuzzy = evalfis(fuzzy_angle_position_matlab,q_r_position_angle);


Force(i) = evaluate_fuzzy(1)*3.5;

    % position of cart (meter)      
    % speed of cart (m/s)      
    % angle of pendulum (radian)   
    % angle speed (radian/s)  
    % torque (radian/s)

% % T_1=num2str(z2(1), 8);
% % T_2= num2str(z2(2),8);
% % T_3=num2str(q_r_angle,8);
% % T_4= num2str(q_r_angle_velocity,8);
% % T_5=num2str(Force(i),8);
%  Data = strcat(T_1,'   ;   ',T_2,'   ;   ',T_3,'   ;   ',T_4,'   ;   ',T_5,'   ;   ')
% Info(i)=Data;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   

   
    % save the data
    z2(1) = AIRead(dev,1)/0.15/100; % position of cart (meter)
    z2(2) = 0.01*round(-13.1*AIRead(dev,3)); % speed of cart (m/s)
    z2(3) = -AIRead(dev,2)/0.96*pi/180; % angle of pendulum (radian)

    
%%%%%%%%%%%%%%%%%%%%%% voltage limitation
    if abs(Force(i))>10
       Force(i) = sign(Force(i))*10; 
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%%%%%%%%%%%% apply calculated voltage    
    tic
    while toc<Ts
        DOWriteBit(dev,1,2,1)           % Freischaltung Pendel
        DOWriteBit(dev,1,2,0)           % channel 1 besteht aus DO0..DO7
        DOWriteBit(dev,1,2,1)           % DO2 benötigt kontinuierlichen Impuls
        AOWrite(dev, 2, Force(i));      % apply calculated voltage
    end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
 
 %%%%%% angular speed calculation (derivative) 
    z2_winkel = -AIRead(dev,2)/0.96*pi/180;
    z2(4) = (z2_winkel-z2(3))/Ts; % angular speed of pendulum
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    


      
 
    
    
    x(i+1,:) = z2';
    
%         if abs(z2(1)) > 0.3     % Der Pendel ist ausser Bereich
%         disp('Please bring me back !');
%         pause(3);                 % wait 3 second
%      end
    
    
    if abs(z2(1)) > 0.5 || abs(z2(3)*180/pi) > 10    % Der Pendel ist ausser Bereich
        disp('Please bring me back !');
        pause(3);                 % wait 3 second
 end
    
end
