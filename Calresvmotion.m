function [Para_opt,Base_pos,Sixgle_pos,Upper_pos] = Calresvmotion(R,r,D,d,Center_pos,Center_pos0,Base_H,Center_H,Angle_RX,Angle_RY,Angle_RZ,G_bar,G_c)
%   三角盘的反解工程
%   此处显示详细说明
%   对结构简图进行复现
%   r为上面盘的半径,d是上面盘的弦长,Center_pos为上面盘的圆心
%   R是下面盘的半径,D是下面盘的弦长,Center_pos0为下面盘的圆心
%   Angle_Pitch为俯仰角, Angle_Yaw为偏航角，Angle_rotate为旋转角
%   G_bar为各个杆的质量，G_c为负载的质量
%% 位置和长度解算
    [Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLengthold(R,r,D,d,Center_pos,Center_pos0,Base_H,Center_H,Angle_RX,Angle_RY,Angle_RZ);    
%%  计算缸的推力
%  Bar_Arrow是杆件的行程信息
    Dxyz_mat = Sixgle_pos - Base_pos;   % 杆的行程数据
    Dxyz_matC = Sixgle_pos - ones(6,1) * Center_pos;   % 杆顶端距离动平面中心的向量
    K1_mat = Dxyz_mat'./(Para_opt(:,1)*ones(1,3))';
    K2_mat = zeros(6,1);
    for j = 1:6
        K2_mat(j,1) = Dxyz_mat(j,3) * G_bar(j)/Para_opt(j,1);
    end
    K2_vec = [0;0;0];
    for j = 1:6
        K2_vec(1) = K2_vec(1) - (Dxyz_mat(j,1) * Dxyz_mat(j,3) * G_bar(j))/Para_opt(j,1)^2;
        K2_vec(2) = K2_vec(2) - (Dxyz_mat(j,2) * Dxyz_mat(j,3) * G_bar(j))/Para_opt(j,1)^2;
        K2_vec(3) = K2_vec(3) - (Dxyz_mat(j,3) * Dxyz_mat(j,3) * G_bar(j))/Para_opt(j,1)^2;
    end
    K3_vec = [0;0;0];
    for j = 1:6
        K3_vec(1) = K3_vec(1) + (Para_opt(j,4) * Dxyz_mat(j,1) * Dxyz_mat(j,3) * G_bar(j))/Para_opt(j,1)^3;
        K3_vec(2) = K3_vec(2) + (Para_opt(j,4) * Dxyz_mat(j,2) * Dxyz_mat(j,3) * G_bar(j))/Para_opt(j,1)^3;
        K3_vec(3) = K3_vec(3) + (Para_opt(j,4) * (Dxyz_mat(j,3) * Dxyz_mat(j,3) - Para_opt(j,1) * Para_opt(j,1)) * G_bar(j))/Para_opt(j,1)^3;
    end
    K3_vec(3) = K3_vec(3) - G_c;
    K01_mat = zeros(3,6);
    for j = 1:6
        K01_mat(1,j) = (Dxyz_matC(j,2) * Dxyz_mat(j,3) - Dxyz_matC(j,3) * Dxyz_mat(j,2))/Para_opt(j,1);
        K01_mat(2,j) = (Dxyz_matC(j,3) * Dxyz_mat(j,1) - Dxyz_matC(j,1) * Dxyz_mat(j,3))/Para_opt(j,1);
        K01_mat(3,j) = (Dxyz_matC(j,1) * Dxyz_mat(j,2) - Dxyz_matC(j,2) * Dxyz_mat(j,1))/Para_opt(j,1);
    end
    K03_vec = [0;0;0];
    for j = 1:6
        K03_vec(1) = K03_vec(1) - (Para_opt(j,4)/Para_opt(j,1)^3) * G_bar(j) * (Dxyz_matC(j,2) * (Dxyz_mat(j,3)^2 - Para_opt(j,1)^2) - Dxyz_matC(j,3) * Dxyz_mat(j,2) * Dxyz_mat(j,3));
        K03_vec(2) = K03_vec(2) - (Para_opt(j,4)/Para_opt(j,1)^3) * G_bar(j) * (Dxyz_matC(j,3) * Dxyz_mat(j,1) * Dxyz_mat(j,3) - Dxyz_matC(j,1) * (Dxyz_mat(j,3)^2 - Para_opt(j,1)^2));
        K03_vec(3) = K03_vec(3) - (Para_opt(j,4)/Para_opt(j,1)^3) * G_bar(j) * (Dxyz_matC(j,1) * Dxyz_mat(j,2) * Dxyz_mat(j,3) - Dxyz_matC(j,2) * Dxyz_mat(j,1) * Dxyz_mat(j,3));
    end
    K4_vec = [0;0;0];
    K04_vec = [0;0;0];
    P_F = inv([K1_mat;K01_mat])*([K1_mat;K01_mat]*K2_mat + [K3_vec;K03_vec] + [K4_vec;K04_vec]); % 获得输出的力
    %figure(1)
    %hold on 
    %fill3(Upper_pos(:,1)',Upper_pos(:,2)',Upper_pos(:,3)','r','EdgeColor','none');
%%  基于六杆参数绘制图形
    %figure(2)
    %hold on
    %for j = 1:6
    %    scatter3(Upper_pos(j,1),Upper_pos(j,2),Upper_pos(j,3),'k.')
    %end
    draw_six_axis_realtime(R, r, Base_pos, Sixgle_pos, Upper_pos);
end

