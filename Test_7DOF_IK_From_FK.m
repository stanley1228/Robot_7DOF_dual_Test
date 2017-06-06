
clear all
close all
clc



%�T�w�Ѽ�
L0=255;     %�i��n�R��
L1=250;     %upper arm
L2=250;   %forearm
L3=150;      %length of end effector
x_base=0;   %����I
y_base=0;
z_base=0;


DEF_DESCRETE_POINT=40;

 
 
 Path=zeros(DEF_DESCRETE_POINT,3);%�W�e�����|�I
 PathPoint=zeros(DEF_DESCRETE_POINT,3);%�O����ڤW���I�A�e�Ϩϥ�
 PathTheta=zeros(DEF_DESCRETE_POINT,7);%�O���C�b���סA�e�Ϩϥ�


DEF_DEG_2_RAD =(pi/180);
DEF_RAD_2_DEG =(180/pi);

%����FK =>IK=>FK=>�e��=>�ݲĤ@�b������ɷ|���|����������180�����D  A:��  shoulder �Mwrist �@���u��

 
for t=1:1:DEF_DESCRETE_POINT
 
     %��J�Ѽ�
    theta=zeros(1,7);
    x_base=0;
    y_base=0;
    z_base=0;
  
     %forward kinematic
    theta(1)=1*t*DEF_DEG_2_RAD;
  
    if t==13
        t=t;
    end    
    theta(2)=0*DEF_DEG_2_RAD;
    theta(3)=0*DEF_DEG_2_RAD;
    theta(4)=-30*DEF_DEG_2_RAD;
    theta(5)=0*DEF_DEG_2_RAD;
    theta(6)=0*DEF_DEG_2_RAD;
    theta(7)=0*DEF_DEG_2_RAD;
    
    %Forward kinematic
    [out_x_end,out_y_end,out_z_end,out_alpha,out_beta,out_gamma,P,RotationM] = FK_7DOF(L1,L2,L3,x_base,y_base,z_base,theta)
%     deg_alpha =out_alpha*DEF_RAD_2_DEG
%     deg_beta=out_beta*DEF_RAD_2_DEG
%     deg_gamma=out_gamma*DEF_RAD_2_DEG;
    
    %Inverse kinematic
    x_end=out_x_end;
    y_end=out_y_end;
    z_end=out_z_end;
    
    in_alpha=out_alpha;
    in_beta=out_beta;
    in_gamma=out_gamma;
    Rednt_alpha=0*DEF_DEG_2_RAD;
    
    theta = IK_7DOF(L1,L2,L3,x_base,y_base,z_base,x_end,y_end,z_end,in_alpha,in_beta,in_gamma,Rednt_alpha);
   
    
     %Forward kinematic
    [out_x_end,out_y_end,out_z_end,out_alpha,out_beta,out_gamma,P,RotationM] = FK_7DOF(L1,L2,L3,x_base,y_base,z_base,theta)
    
   
    %�O�����|�W���I
    PathPoint(t,1:3)=[out_x_end out_y_end out_z_end];
    
    %�e���`�I��
    AZ=0;
    EL=0;
    Draw_7DOF_point(P,RotationM,PathPoint);

    %�O���C�b�����ܤ�
    PathTheta(t,1:7)=theta*(180/pi);
  
%     In=[in_x_end in_y_end in_z_end in_alpha in_beta in_gamma];
%     Out=[out_x_end out_y_end out_z_end out_alpha out_beta out_gamma];
    
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