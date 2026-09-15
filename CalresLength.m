function [Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLength(R,r,D,d,Base_H,Center_H,Upper_Radis,UpperJ_Radis,Cen_X,Cen_Y,Angle_RX,Angle_RY,Angle_RZ,X_Mov,Y_Mov,Z_Mov)
%   三角盘的反解工程
%   此处显示详细说明
%   对结构简图进行复现
%   r为上面盘的半径,d是上面盘的弦长,Center_pos为上面盘的圆心
%   R是下面盘的半径,D是下面盘的弦长,Base_H为下面盘的圆心高度
%   Angle_Pitch为俯仰角, Angle_Yaw为偏航角，Angle_rotate为旋转角
%   G_bar为各个杆的质量，G_c为负载的质量
%   输出量:Base_pos为底盘铰点, Sixgle_pos为顶盘铰点, Upper_pos为顶盘上平台的端点
%   输出量:Para_opt为输出六杆长度、姿态等各个参数矩阵
%% 计算上平台六个顶点的坐标
    Rotate_Center = [Cen_X,Cen_Y,0];
    Center_pos = [X_Mov + Rotate_Center(1), Y_Mov + Rotate_Center(2) - UpperJ_Radis , Z_Mov + Rotate_Center(3)];
    Center_pos0 = [X_Mov + Rotate_Center(1), Y_Mov + Rotate_Center(2) -  Upper_Radis, Z_Mov + Rotate_Center(3)];
    a_theta = asin(d/(2*r)) * 180 / pi();
    Upper_angle = 0 - [-a_theta,a_theta,120 - a_theta,120 + a_theta,240 - a_theta,240 + a_theta];
    r_Mat = NaN * ones(6,3);
    for i = 1:6
        r_Mat(i,1) = r * cosd(Upper_angle(i));
        r_Mat(i,2) = 0;
        r_Mat(i,3) = r * sind(Upper_angle(i));
    end
    Sixgle_pos = NaN * ones(6,3);
    Upper_pos = NaN * ones(6,3);
    for i = 1:6
        Sixgle_pos(i,:) = Center_pos + r_Mat(i,:);
        Upper_pos(i,:) = Center_pos0 + r_Mat(i,:);
    end
%% 进行矩阵旋转
    Rx_mat = [1, 0, 0; 0, cosd(Angle_RX), -sind(Angle_RX); 0, sind(Angle_RX),  cosd(Angle_RX)];
    Ry_mat = [cosd(Angle_RY), 0, sind(Angle_RY); 0, 1, 0; -sind(Angle_RY), 0, cosd(Angle_RY)];
    Rz_mat = [cosd(Angle_RZ), -sind(Angle_RZ), 0; sind(Angle_RZ), cosd(Angle_RZ), 0; 0, 0, 1];
    Rotation_mat = Rx_mat * Ry_mat * Rz_mat;
    for i = 1:6
        Sixgle_pos(i,:) = Center_pos + (Rotation_mat * r_Mat(i,:)')';
        Upper_pos(i,:) = Center_pos0 + (Rotation_mat * r_Mat(i,:)')';
    end
    
%% 计算基座六个点的坐标
    b_theta = asin(D/(2*R)) * 180 / pi();
    Base_angle = 0 - [b_theta - 60,60 - b_theta,60 + b_theta,180 - b_theta,180 + b_theta, 300 - b_theta];
    Base_pos = NaN * ones(6,3);
    for i = 1:6
        Base_pos(i,1) = R * cosd(Base_angle(i));
        Base_pos(i,2) = -(Center_H - Base_H);
        Base_pos(i,3) = R * sind(Base_angle(i));   % 厚度
    end
%% 计算六个杆的参数
    Para_opt = NaN * ones(6,4);
    for j = 1:6
        Bar_Arrow = Sixgle_pos(j,:) - Base_pos(j,:);
        Para_opt(j,1) = sqrt(Bar_Arrow(1)^2 + Bar_Arrow(2)^2 + Bar_Arrow(3)^2);
        if(Bar_Arrow(1) > 0)
            Para_opt(j,2) = atan(Bar_Arrow(2)/Bar_Arrow(1));
        else
            Para_opt(j,2) = atan(Bar_Arrow(2)/Bar_Arrow(1)) + pi();
        end
        Para_opt(j,3) = atan(Bar_Arrow(3)/sqrt(Bar_Arrow(1)^2 + Bar_Arrow(2)^2));
    end
    Para_opt(:,2:3) = Para_opt(:,2:3) * 180/pi();
    Para_opt(:,4) = Para_opt(:,1) - 894.5;  % 液压缸到杆的底部距离
end

