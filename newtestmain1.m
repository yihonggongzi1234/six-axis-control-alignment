%% 执行云台与滑轨导槽的对齐运算
% 输入为9个参数(六轴台的结构参数)
% 可变的变量为六轴台的6个参数(可变输入)
% 求解的参数为云台的两个旋转参数 + 机械臂的六个参数（旋转）
% 获得两轴的夹角和最短距离
%% 0 初始设定
%% 0.1 录入结构整体参数（9+2个）
clear
R = 1425.87;
r = 492.44;
D = 310;
d = 250;
Upper2gd = 2524.85;   %5
Upper_Radis = 1000;   %6
Lower2bt = 236;   %7
Upper2ht = 138;   %8
H_iron = 0;   %9
Cen_X = 100;   %10
Cen_Y = 0;   %11

%% 1. 获取六轴台和导轨的角度及位置
%% 1.0.1 计算杆的行程
[Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLength(R,r,D,d,Lower2bt+H_iron,Upper2gd+Upper_Radis,Upper_Radis,Upper_Radis+Upper2ht,0,0,0,0,0,0,0,0);
L_0 = Para_opt(1,1);
L_max = 0;
for X_Mov = -100:100:100
    for Y_Mov = -100:100:100
        Angle_RX = 30;
        Angle_RY = 0;
        Angle_RZ = 0;
        Z_Mov = 0;
        [Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLength(R,r,D,d,Lower2bt+H_iron,Upper2gd+Upper_Radis,Upper_Radis,Upper_Radis+Upper2ht,Cen_X,Cen_Y,Angle_RX,Angle_RY,Angle_RZ,X_Mov,Y_Mov,Z_Mov);
        L_max = max(L_max,max(Para_opt(:,1)));
        Angle_RX = 0;
        Angle_RY = 30;
        Angle_RZ = 0;
        Z_Mov = 0;
        [Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLength(R,r,D,d,Lower2bt+H_iron,Upper2gd+Upper_Radis,Upper_Radis,Upper_Radis+Upper2ht,Cen_X,Cen_Y,Angle_RX,Angle_RY,Angle_RZ,X_Mov,Y_Mov,Z_Mov);
        L_max = max(L_max,max(Para_opt(:,1)));
    end
end
L_deta = L_max - L_0;  %% 杆的行程范围
%% 1.0.2 计算极端位置
r = 588;
Angle_RX = 0;
Angle_RY = 30;
Angle_RZ = asin(d/(2*r));
X_Mov = 100;
Y_Mov = 100;
Z_Mov = 0;
[Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLength(R,r,D,d,Lower2bt+H_iron,Upper2gd+Upper_Radis,Upper_Radis,Upper_Radis+Upper2ht,Cen_X,Cen_Y,Angle_RX,Angle_RY,Angle_RZ,X_Mov,Y_Mov,Z_Mov);
X_maxrange = max(Upper_pos(:,1));  %% 最大半径
%% 1.1 输入区（控制输入）
r = 492.44;
TT_vec = [0:1:1800]'/200;
UP_vec = zeros(1801,1);
TURN_vec = zeros(1801,1);
UPrail_vec = zeros(1801,1);
TURNrail_vec = zeros(1801,1);
%% 1.1.1 六轴的输入区
for TT = 0:1800
    TT
    Angle_RX = 0;
    Angle_RY = 0;
    Angle_RZ = 30 * sin(0.02*TT);
    X_Mov = 0;
    Y_Mov = 0;
    Z_Mov = 0;
%% 1.1.2 导轨的输入区
    Radis_R = 2000;  % 导轨半径
    Rail_dir = 45 - TT/20;   % 导轨方位
    H_t = -300;  % 设置导轨高度
%% 1.2 计算转台特征
    [Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLength(R,r,D,d,Lower2bt+H_iron,Upper2gd+Upper_Radis,Upper_Radis,Upper_Radis+Upper2ht,Cen_X,Cen_Y,Angle_RX,Angle_RY,Angle_RZ,X_Mov,Y_Mov,Z_Mov);
    L = Para_opt(:,1);
%% 1.3 计算导轨特征
    Robot_ori = [Radis_R * sin(Rail_dir * pi()/180), Radis_R * cos(Rail_dir * pi()/180), H_t];  % 中心点位置

%% 2 第一次初步对焦
%% 2.1 获取云台上筒的方程
    yuntai_RX = 0;
    yuntai_RY = 0;
    EPS = 10000;
    loop_id = 0;
    while((EPS > 0.01) || (loop_id < 5))
        loop_id = loop_id + 1;
        [Global_ori,Global_oridir] = Calyuntaidire(Upper_pos, Angle_RX, Angle_RY, Angle_RZ, yuntai_RX, yuntai_RY);
%% 2.2 计算两点连线的增量方程
        Theta_ori = Robot_ori - Global_ori;
        Theta_norm = sqrt(Theta_ori(1)^2 + Theta_ori(2)^2 + Theta_ori(3)^2);
        Theta_dir = (1/Theta_norm) * Theta_ori;
%% 2.3 计算云台方程
        ytbase_dir = rotateXfuc(rotateYfuc(rotateZfuc([0,1,0], Angle_RZ),Angle_RY),Angle_RX);
        ytvert_dir = rotateXfuc(rotateYfuc(rotateZfuc([1,0,0], Angle_RZ),Angle_RY),Angle_RX);
        ytturn_dir = rotateXfuc(rotateYfuc(rotateZfuc([0,0,1], Angle_RZ),Angle_RY),Angle_RX);
        Vert_axis = [ytvert_dir',ytbase_dir',ytturn_dir'];
        Theta_diryun = (Vert_axis^(-1) * Theta_dir')';
        UP_angle = atan(Theta_diryun(3)/sqrt(Theta_diryun(1)^2 + Theta_diryun(2)^2)) * 180/pi();
        TURN_angle = atan(Theta_diryun(1)/Theta_diryun(2)) * 180/pi();
%% 2.4 计算轨道方程
        Theta_dirrail = -rotateZfuc(Theta_dir, -Rail_dir);
        UPrail_angle = atan(Theta_dirrail(3)/sqrt(Theta_dirrail(1)^2 + Theta_dirrail(2)^2)) * 180/pi();
        TURNrail_angle = atan(Theta_dirrail(1)/Theta_dirrail(2)) * 180/pi();
%% 2.5 更新云台参数
        yuntai_RX = UP_angle;   % 俯仰
        yuntai_RY = TURN_angle;   % 旋转
%% 2.6 检查对齐的情况
%[Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLength(R,r,D,d,Lower2bt+H_iron,Upper2gd+Upper_Radis,Upper_Radis,Upper_Radis+Upper2ht,Cen_X,Cen_Y,Angle_RX,Angle_RY,Angle_RZ,X_Mov,Y_Mov,Z_Mov);
        [Global_ori,Global_oridir] = Calyuntaidire(Upper_pos, Angle_RX, Angle_RY, Angle_RZ, UP_angle, TURN_angle);  % 云台方程
        [Robot_ori,Robot_dir] = CalRaildire(H_t, Rail_dir, UPrail_angle, TURNrail_angle);
        EPS = norm(Global_oridir + Robot_dir);
    end
    UP_vec(TT+1,1) = UP_angle;
    TURN_vec(TT+1,1) = TURN_angle;
    UPrail_vec(TT+1,1) = UPrail_angle;
    TURNrail_vec(TT+1,1) = TURNrail_angle;
end
figure(1)
hold on
title('六轴台上云台俯仰角度')
plot(TT_vec,UP_vec)
xlabel('秒')
ylabel('角度')
figure(2)
hold on
title('六轴台上云台方位角度')
plot(TT_vec,TURN_vec)
xlabel('秒')
ylabel('角度')
figure(3)
hold on
title('升降台上云台俯仰角度')
plot(TT_vec,UPrail_vec)
xlabel('秒')
ylabel('角度')
figure(4)
hold on
title('升降台上云台方位角度')
plot(TT_vec,TURNrail_vec)
xlabel('秒')
ylabel('角度')
UPV_vec = (UP_vec(2:1801,1) - UP_vec(1:1800,1)) * 200;
UPrailV_vec = (UPrail_vec(2:1801,1) - UPrail_vec(1:1800,1)) * 200;
TURNV_vec = (TURN_vec(2:1801,1) - TURN_vec(1:1800,1)) * 200;
TURNrailV_vec = (TURNrail_vec(2:1801,1) - TURNrail_vec(1:1800,1)) * 200;
TTV_rec = (TT_vec(2:1801) + TT_vec(1:1800))/2;
figure(5)
hold on
title('六轴台上云台俯仰速度')
plot(TTV_rec,UPV_vec)
xlabel('秒')
ylabel('角度/秒')
figure(6)
hold on
title('六轴台上云台滚转速度')
xlabel('秒')
ylabel('角度/秒')
plot(TTV_rec,TURNV_vec)
figure(7)
hold on
title('升降台上云台俯仰速度')
plot(TTV_rec,UPrailV_vec)
xlabel('秒')
ylabel('角度/秒')
figure(8)
hold on
title('升降台上云台方位速度')
plot(TTV_rec,TURNrailV_vec)
xlabel('秒')
ylabel('角度/秒')
%% 旋转功能函数
function new_vec = rotateZfuc(old_vec, theta)
    new_vec = [cosd(theta),  sind(theta), 0; -sind(theta),  cosd(theta), 0; 0, 0, 1] * old_vec';
    new_vec = new_vec';
end

function new_vec = rotateYfuc(old_vec, theta)
    new_vec = [cosd(theta), 0, sind(theta); 0, 1, 0; -sind(theta), 0, cosd(theta)] * old_vec';
    new_vec = new_vec';
end

function new_vec = rotateXfuc(old_vec, theta)
    new_vec = [1, 0, 0; 0, cosd(theta), -sind(theta); 0, sind(theta),  cosd(theta)] * old_vec';
    new_vec = new_vec';
end

function R = axisAngleRotation(vert_norm, theta)
% axisAngleRotation  绕任意轴(a,b,c)旋转theta角度的旋转矩阵
% 输入:
%   a, b, c : 旋转轴方向分量
%   theta   : 旋转角（弧度）
% 输出:
%   R       : 3×3旋转矩阵

    % Step 1: 归一化旋转轴
    u = vert_norm / norm(vert_norm);  % 确保为单位向量
    a = u(1); b = u(2); c = u(3);

    % Step 2: 计算正弦余弦
    ct = cosd(theta);
    st = sind(theta);

    % Step 3: 罗德里格斯公式展开
    R = [ ct + a^2*(1-ct),    a*b*(1-ct) - c*st,  a*c*(1-ct) + b*st;
          b*a*(1-ct) + c*st,  ct + b^2*(1-ct),    b*c*(1-ct) - a*st;
          c*a*(1-ct) - b*st,  c*b*(1-ct) + a*st,  ct + c^2*(1-ct) ];
end

function [Global_ori,Global_oridir] = Calyuntaidire(Upper_pos, Angle_RX, Angle_RY, Angle_RZ, up_angle, turn_angle)
%% 2.2 计算云台坐标和方向
%转台上顶面的坐标
    Upper_cent = [mean(Upper_pos(:,1)),mean(Upper_pos(:,2)),mean(Upper_pos(:,3))];
% 计算云台的dx,dy和dz三个量
    yuntaidx = [1,0,0];
    yuntaidy = [0,1,0];
    yuntaidz = [0,0,1];
    yuntaidx = rotateXfuc(rotateYfuc(rotateZfuc(yuntaidx, Angle_RZ),Angle_RY),Angle_RX);
    yuntaidy = rotateXfuc(rotateYfuc(rotateZfuc(yuntaidy, Angle_RZ),Angle_RY),Angle_RX);
    yuntaidz = rotateXfuc(rotateYfuc(rotateZfuc(yuntaidz, Angle_RZ),Angle_RY),Angle_RX);
% 球体的中心坐标
    Global_cent = Upper_cent - yuntaidz * 820;
%% 2.3 计算云台发射源和发射方向
% 球体偏置的坐标（注意转序问题）
    yuntaiDX = (axisAngleRotation(yuntaidx, up_angle)*yuntaidx')';   % 垂直旋转
    yuntaiDY = (axisAngleRotation(yuntaidx, up_angle)*yuntaidy')';   % 垂直旋转
    yuntaiDZ = (axisAngleRotation(yuntaidx, up_angle)*yuntaidz')';   % 垂直旋转

    yuntaiDX = (axisAngleRotation(yuntaidz, -turn_angle)*yuntaiDX')';   % 水平旋转
    yuntaiDY = (axisAngleRotation(yuntaidz, -turn_angle)*yuntaiDY')';   % 水平旋转
    yuntaiDZ = (axisAngleRotation(yuntaidz, -turn_angle)*yuntaiDZ')';   % 水平旋转

    yuntaiDX = yuntaiDX/norm(yuntaiDX);
    yuntaiDY = yuntaiDY/norm(yuntaiDY);
    yuntaiDZ = yuntaiDZ/norm(yuntaiDZ);
    Global_ori = Global_cent + 350 * yuntaiDY + 200 * yuntaiDX - 200 * yuntaiDZ;  % 发射源
    Global_oridir = yuntaiDY;     % 发射方向
end


function [Robot_ori,Robot_dir] = CalRaildire(H_t, Rail_dir, Up_angle, Turn_angle)
%% 3.2 核心参数设定
    Radis_R = 2000;  % 计算半径
    Robot_ori = [Radis_R * sin(Rail_dir * pi()/180), Radis_R * cos(Rail_dir * pi()/180), H_t];  % 中心点位置
    robotdx = [-1,0,0];
    robotdy = [0,-1,0];
    robotdz = [0,0,1];
    robotdx = rotateZfuc(robotdx, Rail_dir);
    robotdy = rotateZfuc(robotdy, Rail_dir);
    robotdz = rotateZfuc(robotdz, Rail_dir);
    Robot_dir = (axisAngleRotation(robotdx, Up_angle)*robotdy')';   % 垂直旋转
    Robot_dir = (axisAngleRotation(robotdz, -Turn_angle)*Robot_dir')';   % 水平旋转
end
