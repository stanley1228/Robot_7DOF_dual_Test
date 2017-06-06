
clear all
close all
clc



%�T�w�Ѽ�
L0=0;     %�i��n�R��
L1=100;     %upper arm
L2=100;   %forearm
L3=10;      %length of end effector
x_base=0;   %����I
y_base=0;
z_base=0;




%P2P
 O=[0 0 -(L1+L2+L3)]; %��l�I
 S=[0 -20 -180]; %�Ĥ@�b���|������ౡ�p�����I���|

 %O=[20 20 0]; %��l�I
% S=[20 -20 0]; %�Ĥ@�b���|������ౡ�p�����I���|
 
 
 in_alpha=0*(pi/180);
 in_beta=0*(pi/180);
 in_gamma=(90)*(pi/180);
 Rednt_alpha=-(90)*(pi/180);
 
 %inverse kinematic
 theta_O=IK_7DOF_stanley(L1,L2,L3,x_base,y_base,z_base,O(1),O(2),O(3),in_alpha,in_beta,in_gamma,Rednt_alpha);
 theta_S=IK_7DOF_stanley(L1,L2,L3,x_base,y_base,z_base,S(1),S(2),S(3),in_alpha,in_beta,in_gamma,Rednt_alpha);
 
 
 
 angle_offset=theta_S-theta_O;
 Norm_angle=norm(angle_offset);
 Max_angular_velocity=0.1;
 TimeNeed=Norm_angle/Max_angular_velocity;
 
 PathTheta=zeros(round(TimeNeed),7);%�O���C�b���סA�e�Ϩϥ�
 
 for t=1:1:round(TimeNeed)
        for i=1:1:7
            PathTheta(t,i)=Max_angular_velocity*(t-1)*angle_offset(1,i)/Norm_angle+theta_O(1,i);
        end
 end

 
 %�eJointAngle
 Draw_7DOF_JointAnglePath(PathTheta);

 
 
 
 