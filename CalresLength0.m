function [Len_Para] = CalresLength0(R,r,D,d,Base_H,Center_H,Upper_Radis,UpperJ_Radis,Angle_Pitch,Angle_Yaw,Angle_rotate,X_Mov,Y_Mov,Z_Mov)
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
    Rotate_Center = [0,0,Center_H];
    Center_pos = [X_Mov + Rotate_Center(1) - UpperJ_Radis * sind(Angle_Pitch), Y_Mov + Rotate_Center(2) - UpperJ_Radis * cosd(Angle_Pitch) * sind(Angle_Yaw), Z_Mov + Rotate_Center(3) - UpperJ_Radis * cosd(Angle_Pitch) * cosd(Angle_Yaw)];
    Center_pos0 = [X_Mov + Rotate_Center(1) - Upper_Radis * sind(Angle_Pitch), Y_Mov + Rotate_Center(2) - Upper_Radis * cosd(Angle_Pitch) * sind(Angle_Yaw), Z_Mov + Rotate_Center(3) - Upper_Radis * cosd(Angle_Pitch) * cosd(Angle_Yaw)];
    a_theta = asin(d/r) * 180 / pi();
    Upper_angle = Angle_rotate + [-a_theta,a_theta,120 - a_theta,120 + a_theta,240 - a_theta,240 + a_theta];
    r_Mat = NaN * ones(6,2);
    for i = 1:6
        r_Mat(i,1) = r * cosd(Upper_angle(i));
        r_Mat(i,2) = r * sind(Upper_angle(i));
    end
    r_Mat = r_Mat';
    
    Norm_vert(1) = sind(Angle_Pitch);
    Norm_vert(2) = cosd(Angle_Pitch) * sind(Angle_Yaw);
    Norm_vert(3) = cosd(Angle_Pitch) * cosd(Angle_Yaw);
    S_1 = [Norm_vert(3)/sqrt(Norm_vert(1)^2 + Norm_vert(3)^2),0,-Norm_vert(1)/sqrt(Norm_vert(1)^2 + Norm_vert(3)^2)];
    S_2 = cross(Norm_vert,S_1);
    S_2 = S_2/sqrt(S_2(1)^2 + S_2(2)^2 + S_2(3)^2);
    Sixgle_pos = NaN * ones(6,3);
    Upper_pos = NaN * ones(6,3);
    for i = 1:6
        Sixgle_pos(i,:) = Center_pos + r_Mat(1,i) * S_1 + r_Mat(2,i) * S_2;
        Upper_pos(i,:) = Center_pos0 + r_Mat(1,i) * S_1 + r_Mat(2,i) * S_2;
    end
%% 计算基座六个点的坐标
    b_theta = asin(D/R) * 180 / pi();
    Base_angle = [-60 + b_theta,60 - b_theta,60 + b_theta,180 - b_theta,180 + b_theta, 300 - b_theta];
    Base_pos = NaN * ones(6,3);
    for i = 1:6
        Base_pos(i,1) = R * cosd(Base_angle(i));
        Base_pos(i,2) = R * sind(Base_angle(i));
        Base_pos(i,3) = Base_H;   % 厚度
    end
%% 计算六个杆的参数
    Len_Para = NaN * ones(6,1);
    for j = 1:6
        Bar_Arrow = Sixgle_pos(j,:) - Base_pos(j,:);
        Len_Para(j,1) = sqrt(Bar_Arrow(1)^2 + Bar_Arrow(2)^2 + Bar_Arrow(3)^2);
    end
end

