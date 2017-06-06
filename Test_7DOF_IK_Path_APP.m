
clear all
close all
clc



%�T�w�Ѽ�
L0=225;     %�i��n�R��
L1=250;     %upper arm
L2=250;   %forearm
L3=150;      %length of end effector
x_base=0;   %����I
y_base=0;
z_base=0;

DEF_DESCRETE_POINT=90;

%{
P7=[71.3397;-5.0000;0]
P8IK=[77.2464;3.069;0]
P8=[80;0;0]
V_r_hIK=[P8IK-P7]
norm(V_r_hIK)

V_r=[P8-P7]
theta1=acos(V_r_hIK'*V_r/(norm(V_r_hIK)*norm(V_r)))
theta1=theta1*(180)/pi
%}
 
 %���ͥ����O(60,0,0)  Q(80,-20,0)  R(80,-20,-20)  S(60,0,-20)
 %�⦹���|����90��
 O=[500 220 0];
 Q=[500 0 0];
 R=[500 0 -220];
 S=[500 220 -220];
 
 Path=zeros(DEF_DESCRETE_POINT,3);%�W�e�����|�I
 PathPoint=zeros(DEF_DESCRETE_POINT,3);%�O����ڤW���I�A�e�Ϩϥ�
 PathTheta=zeros(DEF_DESCRETE_POINT,7);%�O���C�b���סA�e�Ϩϥ�

 %�e����ΰ�IK FK����
 for t=1:1:DEF_DESCRETE_POINT
    if t<=25
        Path(t,1:3)=O+(Q-O)*t/25;
    elseif t<=50
        Path(t,1:3)=Q+(R-Q)*(t-25)/25;
    elseif t<=75
         Path(t,1:3)=R+(S-R)*(t-50)/25;
    else 
         Path(t,1:3)=S+(O-S)*(t-75)/15;
    end
 end

%�e���u
%  O=[0 0 -(L1+L2+L3)]; %��l�I
%  Q=[0 -20 -180]; %�Ĥ@�b���|������ౡ�p�����I���|
% 
%  for t=1:1:DEF_DESCRETE_POINT
%         Path(t,1:3)=O+(Q-O)*(t-1)/DEF_DESCRETE_POINT;
%  end

for t=1:1:DEF_DESCRETE_POINT
 
    %��J�Ѽ�
    in_x_end=Path(t,1);
    in_y_end=Path(t,2);
    in_z_end=Path(t,3);
   
    in_alpha=0*(t/DEF_DESCRETE_POINT)*(pi/180);
    in_beta=0*(t/DEF_DESCRETE_POINT)*(pi/180);
    in_gamma=0*(t/DEF_DESCRETE_POINT)*(pi/180);

    Rednt_alpha=-(45)*(pi/180);
    %��X�Ѽ� initial
    %theta=zeros(1,7);
    
    %
    %���I��min==>IK==>theta==>FK==>���I��mout
    %
    %inverse kinematic
    theta=IK_7DOF(L1,L2,L3,x_base,y_base,z_base,in_x_end,in_y_end,in_z_end,in_alpha,in_beta,in_gamma,Rednt_alpha);
    
   
   
    %forward kinematic
    %theta=[0 0 0 0 0 0 0];
    [out_x_end,out_y_end,out_z_end,out_alpha,out_beta,out_gamma,P,RotationM] = FK_7DOF(L1,L2,L3,x_base,y_base,z_base,theta);

    
    %�O�����|�W���I
    PathPoint(t,1:3)=[out_x_end out_y_end out_z_end];
    
    %�e���`�I��
    Draw_7DOF_point(P,RotationM,PathPoint);

    %�O���C�b�����ܤ�
    PathTheta(t,1:7)=theta*(180/pi);
  
    In=[in_x_end in_y_end in_z_end in_alpha in_beta in_gamma]
    Out=[out_x_end out_y_end out_z_end out_alpha out_beta out_gamma]
    
    %�T�{FK �MIK�~�t
%     if(out_x_end-in_x_end)>1e-5 || (out_y_end-in_y_end)>1e-5 || (out_z_end-in_z_end)>1e-5 || (out_alpha-in_alpha)>1e-5 || (out_beta-in_beta)>1e-5 || (out_gamma-in_gamma)>1e-5 
%         display('===============')
%         display('IK FK not match')
%         i
%         In=[in_x_end in_y_end in_z_end in_alpha*(180/pi) in_beta*(180/pi) in_gamma*(180/pi)]
%         Out=[out_x_end out_y_end out_z_end out_alpha*(180/pi) out_beta*(180/pi) out_gamma*(180/pi)]
%         
%         break;
%     end
    
    pause(0.1);
end

 %�eJointAngle
%  Draw_7DOF_JointAnglePath(PathTheta);