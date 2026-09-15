%% 执行云台与机械臂的对齐运算
% 输入为9个参数(六轴台的结构参数)
% 可变的变量为六轴台的6个参数(可变输入)
% 求解的参数为云台的两个旋转参数 + 机械臂的六个参数（旋转）
% 获得两轴的夹角和最短距离
%% 0 初始设定
%% 0.1 录入结构整体参数（9+2个）
R = 1425.87;
r = 752.35;
D = 310;
d = 250;
Upper2gd = 2428.6;   %5
Upper_Radis = 820;   %6
Lower2bt = 213;   %7
Upper2ht = 133;   %8
H_iron = 0;   %9
Cen_X = 0;   %10
Cen_Y = 0;   %11
%% 0.2 录入旋转坐标
Rx_mat = [1, 0, 0; 0, cosd(Angle_RX), -sind(Angle_RX); 0, sind(Angle_RX),  cosd(Angle_RX)];
Ry_mat = [cosd(Angle_RY), 0, sind(Angle_RY); 0, 1, 0; -sind(Angle_RY), 0, cosd(Angle_RY)];
Rz_mat = [cosd(Angle_RZ),  sind(Angle_RZ), 0; -sind(Angle_RZ),  cosd(Angle_RZ), 0; 0, 0, 1];
%% 0.3 录入机械臂参数

%% 1. 获取六轴台的姿态角度
%% 1.1 六轴台的输入区（控制输入）
Angle_RX = 10;
Angle_RY = 10;
Angle_RZ = 0;
X_Mov = 0;
Y_Mov = 0;
Z_Mov = 0;
%% 1.2 计算转台特征
[Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLength(R,r,D,d,Lower2bt+H_iron,Upper2gd+Upper_Radis,Upper_Radis,Upper_Radis+Upper2ht,Cen_X,Cen_Y,Angle_RX,Angle_RY,Angle_RZ,X_Mov,Y_Mov,Z_Mov);
L = Para_opt(:,1);
%% 2. 获取云台上筒的方程
%% 2.1 云台角度的输入区（计算获得的参数）
yuntai_RX = -10;
yuntai_RY = -10;
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
yuntaiDX = rotateYfuc(rotateXfuc(yuntaidx, yuntai_RX),yuntai_RY);
yuntaiDY = rotateYfuc(rotateXfuc(yuntaidy, yuntai_RX),yuntai_RY);
yuntaiDZ = rotateYfuc(rotateXfuc(yuntaidz, yuntai_RX),yuntai_RY);
yuntaiDX = yuntaiDX/norm(yuntaiDX);
yuntaiDY = yuntaiDY/norm(yuntaiDY);
yuntaiDZ = yuntaiDZ/norm(yuntaiDZ);
Global_ori = Global_cent + 350 * yuntaidy + 200 * yuntaidx - 200 * yuntaidz;  % 发射源
Global_oridir = yuntaiDY;     % 发射方向
%% 3. 计算机械臂
%% 3.1 机械臂的输出参数
Detavert = [100,100,100,100,100];   % 沿着坐标轴的偏移量
radisvec = [100,100,100,100,100];   % 旋转半径
Thetavec = [45,90,90,90,90];  % 手臂的角度
converThetavec = [90,90,90,90,0]; % 转轴的角度 
% 沿着旋转轴的偏移
% 沿着旋转轴和旋转半径的旋转
% 坐标系的更改（旋转轴的更改）
robotdx = [1,0,0];
robotdy = [0,1,0];
robotdz = [0,0,1];  % 法线方向
%% 3.2 逐渐获得末端坐标和方向
Pt_pos = [0,3820,-1548.6];
% 第2轴~第6轴
for K = 2:6
    Pt_pos = Pt_pos + robotdz * Detavert(K-1);
    Pt_pos = Pt_pos + radisvec(K-1) * robotdx * cosd(Thetavec(K-1)) + radisvec(K-1) * robotdy * sind(Thetavec(K-1));
    robotdx = (axisAngleRotation(robotdz, Thetavec(K-1)) * robotdx')';
    robotdy = (axisAngleRotation(robotdz, Thetavec(K-1)) * robotdy')';
    robotdz = (axisAngleRotation(robotdz, Thetavec(K-1)) * robotdz')';
    if(K == 2)
        robotdx = (axisAngleRotation(robotdy, converThetavec(K-1)) * robotdx')';
        robotdy = (axisAngleRotation(robotdy, converThetavec(K-1)) * robotdy')';
        robotdz = (axisAngleRotation(robotdy, converThetavec(K-1)) * robotdz')';
    else
        robotdx = (axisAngleRotation(robotdx, converThetavec(K-1)) * robotdx')';
        robotdy = (axisAngleRotation(robotdx, converThetavec(K-1)) * robotdy')';
        robotdz = (axisAngleRotation(robotdx, converThetavec(K-1)) * robotdz')';
    end
end
Robot_ori = Pt_pos;
Robot_dir = robotdy;
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
