R = 2157.10/2;  % 下面盘的半径
r = 1209.70/2;  % 上面盘的半径
D = 300/2;   % 下面盘的弦长
d = 250/2;   % 上面盘的弦长
Base_H = 193;  % 下面台子的高度
Center_H = 2600;  % 上面回转中心的高度
Upper_Radis = 899.2; % 上面台的半径
UpperJ_Radis = 1080.2; % 上铰链的半径
Pitch_Amp = 20;
Yaw_Amp = 0;
Rotate_Amp = 0;
X_Amp = 0;
Y_Amp = 0;
Z_Amp = 0;

Center_pos = [0,0,2620];

[Length_record,Theta_record1,Theta_record2] = calmovfunc(R,r,D,d,Base_H,Center_H,Upper_Radis,UpperJ_Radis,Pitch_Amp,Yaw_Amp,Rotate_Amp,X_Amp,Y_Amp,Z_Amp);

F = 1/(1.3 * pi());
figure(20)
for k = 1:6
    plot([0:1/(200*F):1/F],Length_record(:,k+6),'linewidth',3,'color',[0.15*k,1-0.15*k,0])
    hold on
    title(['随着Z方向控制中心在-260到260mm之间行程,六个杆件的长度变化']);
    axis on
    xlabel('时间序列')
    ylabel('长度数值')
end

figure(3)
V_record = zeros(200,6);
for k = 1:6
    V_record(:,k) = (Length_record(2:201,k+6) - Length_record(1:200,k+6)) * 200 * F;
    plot([0.5/(200*F):1/(200*F):1/F-0.5/(200*F)],V_record(:,k),'linewidth',3,'color',[0.15*k,1-0.15*k,0])
    hold on
    title(['随着Z方向控制中心在-260到260mm之间行程,六个杆件的速度变化']);
    axis on
    xlabel('时间序列')
    ylabel('速度数值')
end

figure(4)
A_record = zeros(199,6);
for k = 1:6
    A_record(:,k) = (V_record(2:200,k) - V_record(1:199,k)) * 200 * F;
    plot([1/(F*200):1/(F*200):1/F-1/(F*200)],A_record(:,k),'linewidth',3,'color',[0.15*k,1-0.15*k,0])
    hold on
    title(['随着Z方向控制中心在-260mm到260mm之间行程,六个杆件的加速度变化']);
    axis on
    xlabel('时间序列')
    ylabel('加速度数值')
end

figure(5)
for k = 1:6
    plot([0:0.005/F:1/F],mod(Theta_record1(:,k+6) - 180,360) - 180,'linewidth',3,'color',[0.15*k,1-0.15*k,0])
    hold on
    title(['随着俯仰角在-20到20度之间摆动,六个杆件的角度变化']);
    axis on
    axis([0,1/F,-180,180])
    xlabel('时间序列')
    ylabel('角度数值')
end