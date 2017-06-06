function [Tacc,Tmax,Tdec,T_all,Vel_axis,Acc_axis,Dec_axis]=VelPlan(theta_org,theta_target,Max_Vel,Max_Acc,Max_Dec)

    delta_theta=theta_target-theta_org;
   
    %find max in delta_theta
    Most_far_AXIS=1;
    for i=2:1:7
        if delta_theta(i) >  delta_theta(Most_far_AXIS)
            Most_far_AXIS=i;
        end   

    end
    
    %�̻��b���̰��t
    Vel_axis(Most_far_AXIS)=Max_Vel;
    Acc_axis(Most_far_AXIS)=Max_Acc;
    Dec_axis(Most_far_AXIS)=Max_Dec;
    
    %�p���Υ[��t�Ϭq�ɶ�
    Tacc=Max_Vel/Max_Acc;
    Tdec=Tacc;
    Tmax=(delta_theta(Most_far_AXIS)-Tacc*Vel_axis(Most_far_AXIS))/Vel_axis(Most_far_AXIS);
    T_all=Tacc+Tmax+Tdec;
    
    
    %�̷өM�̻��b����ҵ��C�b�t�פ�ACC DEC
    for i=1:1:7
        Vel_axis(i)=Max_Vel*(delta_theta(i)/delta_theta(Most_far_AXIS));
        Acc_axis(i)=Vel_axis(i)/Tacc;
        Dec_axis(i)=Vel_axis(i)/Tdec;
    end
  
end