function [Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLengthold(R,r,D,d,Center_pos,Center_pos0,Base_H,Center_H,Angle_RX,Angle_RY,Angle_RZ)
%   三角盘的反解工程
%   对结构简图进行复现
%   r为上面盘的半径,d是上面盘的弦长,Center_pos为上面盘的圆心
%   R是下面盘的半径,D是下面盘的弦长,Base_H为下面盘的圆心高度
%   Angle_Pitch为俯仰角, Angle_Yaw为偏航角，Angle_rotate为旋转角
%   G_bar为各个杆的质量，G_c为负载的质量
%   输出量:Base_pos为底盘铰点, Sixgle_pos为顶盘铰点, Upper_pos为顶盘上平台的端点
%   输出量:Para_opt为输出六杆长度、姿态等各个参数矩阵
%% 计算上平台六个顶点的坐标
    a_theta = asind(d / (2 * r));
%     Upper_angle =  [-a_theta,a_theta,120 - a_theta,120 + a_theta,240 - a_theta,240 + a_theta];
    Upper_angle =  [90-a_theta,90+a_theta,210 - a_theta,210 + a_theta,330 - a_theta,330 + a_theta];
    r_Mat = NaN * ones(6,3);
    for i = 1:6
        r_Mat(i,1) = r * sind(Upper_angle(i));
        r_Mat(i,3) =  r * cosd(Upper_angle(i));
        r_Mat(i,2) = 0;
    end
    Sixgle_pos = NaN * ones(6,3);
    Upper_pos = NaN * ones(6,3);
    Rx_mat = [1, 0, 0; 0, cosd(Angle_RX), -sind(Angle_RX); 0, sind(Angle_RX),  cosd(Angle_RX)];
    Ry_mat = [cosd(Angle_RY), 0, sind(Angle_RY); 0, 1, 0; -sind(Angle_RY), 0, cosd(Angle_RY)];
    Rz_mat = [cosd(Angle_RZ), -sind(Angle_RZ), 0; sind(Angle_RZ), cosd(Angle_RZ), 0; 0, 0, 1];
    Rotation_mat = Rx_mat * Ry_mat * Rz_mat;
    for i = 1:6
        Sixgle_pos(i,:) = Center_pos + (Rotation_mat * r_Mat(i,:)')';
        Upper_pos(i,:) = Center_pos0 + (Rotation_mat * r_Mat(i,:)')';
    end
    
%% 计算基座六个点的坐标
    b_theta = asind(D/(2*R));
    Base_angle = [30+ b_theta,150-b_theta,150+ b_theta,270- b_theta, 270+ b_theta, 30-b_theta];
%     Base_angle = 0 - [b_theta - 60,60 - b_theta,60 + b_theta,180 - b_theta,180 + b_theta, 300 - b_theta];
    Base_pos = NaN * ones(6,3);
    for i = 1:6
        Base_pos(i,3) = R * cosd(Base_angle(i));
        Base_pos(i,1) =  R * sind(Base_angle(i));
        Base_pos(i,2) = -(Center_H - Base_H);   % 厚度
    end
%% 计算六个杆的参数
    Para_opt = NaN * ones(6,4);
    for j = 1:6
        Bar_Arrow = Sixgle_pos(j,:) - Base_pos(j,:);
        Para_opt(j,1) = sqrt(Bar_Arrow(1)^2 + Bar_Arrow(2)^2 + Bar_Arrow(3)^2);
        Para_opt(j,2) = atan2d(Bar_Arrow(2),Bar_Arrow(1));
        Para_opt(j,3) = atan2d(Bar_Arrow(3),sqrt(Bar_Arrow(1)^2 + Bar_Arrow(2)^2));
    end
    Para_opt(:,2:3) = Para_opt(:,2:3) * 180/pi();
    Para_opt(:,4) = Para_opt(:,1) - 894.5;  % 液压缸到杆的底部距离
end

