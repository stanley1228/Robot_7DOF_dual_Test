clear all
close all
clc

theta_org=[0;0;0;0;0;0;0];
theta_target=[60;80;100;-100;-80;-60;50];
theta_now=zeros(1,7);

%MX64 �W��%
DEF_MX64_UNIT_TO_DEG_P_S=0.684;
DEF_MX64_UNIT_TO_RPM=0.114;
DEF_MX64_UNIT_TO_ACC=8.583;  %���L�S��k�]
DEF_MX64_MAX_VELOCITY=702;  %deg/s
DEF_MX64_MAX_ACC=2180;      %deg/s^2

Max_Vel=100;%���w���t�� ���p��W��
Max_Acc=200;%���w���[�t�� ���p��W�� 2180 ���L�]����]�Ӥp�A���i��X�{�٦b�[�t�q�N�w�g�ᱼ�Ҧ��Z���F�A�F����̰��t
Max_Dec=Max_Acc; %�S����t�װѼ�

%===�W��===%
[Tacc,Tmax,Tdec,T_all,Vel_axis,Acc_axis,Dec_axis]=VelPlan(theta_org,theta_target,Max_Vel,Max_Acc,Max_Dec)


%===�}�l�g�����I===%
DEF_PERIOD=0.01;%10ms/cycle
theta_now_record =zeros(round(T_all/DEF_PERIOD),7);
theta_vel_record =zeros(round(T_all/DEF_PERIOD),7);

index=2;
for t=0:DEF_PERIOD:T_all
    theta_now = Period_Point_out(t,theta_org,Vel_axis,Acc_axis,Dec_axis,Tacc,Tmax,Tdec,T_all);
  
    %===�����O���e��===%
    theta_now_record(index,1:7)=theta_now(1:7);
    theta_vel_record(index,1:7)=(theta_now_record(index,1:7)-theta_now_record(index-1,1:7))/DEF_PERIOD;
    index=index+1;
end

%===draw deg===%
figure(1)
cla reset
xlabel('t(10ms)');
ylabel('angle');
hold on; grid on;   
t=1:1:size(theta_now_record,1);

for i=1:1:7
  plot(t,theta_now_record(:,i),'LineWidth',2); 
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

%===draw vel===%
figure(2)
cla reset
xlabel('t(10ms)');
ylabel('vel(deg/s)');
hold on; grid on;   
t=1:1:size(theta_vel_record,1);
for i=1:1:7
  plot(t,theta_vel_record(:,i),'LineWidth',2); 
end

legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');




