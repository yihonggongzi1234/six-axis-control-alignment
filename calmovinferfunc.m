function [Infer_val] = calmovinferfunc(R,r,D,d,Upper2gd,Upper_Radis,Lower2bt,Upper2ht,H_iron,Cen_X,Cen_Y,RX_Amp,RY_Amp,RZ_Amp,X_Amp,Y_Amp,Z_Amp) 
% R:下面盘的半径; r:上面盘的半径; D:下面盘的弦长; d:上面盘的弦长; Upper2gd:动平台静位距地;
% Upper_Radis为上平台的回转半径; Lower2bt: 静铰点距台底面;Upper2ht: 动铰点距动平面; H_iron: 垫铁高度;
% Cen_X回转中心在X方向的平移; Cen_Y回转中心在Y方向的平移
% Pitch_Amp是俯仰角的幅值; Yaw_Amp是偏航角的幅值; Rotate_Amp是滚转角的幅值;
% X_Amp,Y_Amp,Z_Amp分别是X,Y,Z三个方向的幅值
    Infer_val = zeros(41,41,41);   % X方向的-1000到1000，Y方向的-1000到1000，Z方向的200到2200
    Base_H = Lower2bt + H_iron;     % Base_H:下面台子的高度
    Center_H = Upper2gd + Upper_Radis;    % Center_H: 回转中心的高度
    UpperJ_Radis = Upper_Radis + Upper2ht;   % UpperJ_Radis为上铰链的回转半径

    TT = [0:20]';
    Rotate_Center = [Cen_X,Cen_Y,0];
    Max_pos = -10000 * ones(6,3);
    Min_pos = 10000 * ones(6,3);
    RX_rec = sind(0 + 180*TT/10) * RX_Amp;
    RY_rec = sind(0 + 180*TT/10) * RY_Amp;
    RZ_rec = sind(0 + 180*TT/10) * RZ_Amp;
    X_rec = sind(0 + 180*TT/10) * X_Amp;
    Y_rec = sind(0 + 180*TT/10) * Y_Amp;
    Z_rec = sind(0 + 180*TT/10) * Z_Amp;

    Length_record = zeros(21,12);
    Length_record(:,1) = RX_rec;
    Length_record(:,2) = RY_rec;
    Length_record(:,3) = RZ_rec;
    Length_record(:,4) = X_rec;
    Length_record(:,5) = Y_rec;
    Length_record(:,6) = Z_rec;

    Theta_record1 = Length_record;
    Theta_record2 = Length_record;

    b_theta = asin(D/(2*R)) * 180 / pi();
    Base_angle = 0 - [b_theta - 60,60 - b_theta,60 + b_theta,180 - b_theta,180 + b_theta, 300 - b_theta];
    Base_pos = NaN * ones(6,3);
    for i = 1:6
        Base_pos(i,1) = R * cosd(Base_angle(i));
        Base_pos(i,2) = - R * sind(Base_angle(i));
        Base_pos(i,3) = Center_H - Base_H;   % 厚度
    end
    
    for TT = 0:20
        Length_record(TT+1,1) = sind(0 + 180*TT/10) * RX_Amp;
        Theta_record1(TT+1,1) = sind(0 + 180*TT/10) * RY_Amp;
        Theta_record2(TT+1,1) = sind(0 + 180*TT/10) * RZ_Amp;
    end
    for TT = 1:21    
        Center_pos = [X_rec(TT) + Rotate_Center(1), Y_rec(TT) + Rotate_Center(2), Z_rec(TT) + Rotate_Center(3) + UpperJ_Radis];
        Center_pos0 = [X_rec(TT) + Rotate_Center(1), Y_rec(TT) + Rotate_Center(2), Z_rec(TT) + Rotate_Center(3) + Upper_Radis];
    %% 开始计算
        a_theta = asin(d/(2*r)) * 180 / pi();
        Upper_angle = 0 - [-a_theta,a_theta,120 - a_theta,120 + a_theta,240 - a_theta,240 + a_theta];
        r_Mat = NaN * ones(6,3);
        for i = 1:6
            r_Mat(i,1) = r * cosd(Upper_angle(i));
            r_Mat(i,2) = - r * sind(Upper_angle(i));
            r_Mat(i,3) = 0;
        end
        Sixgle_pos = NaN * ones(6,3);
        Upper_pos = NaN * ones(6,3);
        for i = 1:6
            Sixgle_pos(i,:) = Center_pos + r_Mat(i,:);
            Upper_pos(i,:) = Center_pos0 + r_Mat(i,:);
        end
%% 进行矩阵旋转
        Rx_mat = [1, 0, 0; 0, cosd(RX_rec(TT)), -sind(RX_rec(TT)); 0, sind(RX_rec(TT)),  cosd(RX_rec(TT))];
        Ry_mat = [cosd(RY_rec(TT)), 0, sind(RY_rec(TT)); 0, 1, 0; -sind(RY_rec(TT)), 0, cosd(RY_rec(TT))];
        Rz_mat = [cosd(RZ_rec(TT)),  sind(RZ_rec(TT)), 0; -sind(RZ_rec(TT)),  cosd(RZ_rec(TT)), 0; 0, 0, 1];
        for i = 1:6
            Sixgle_pos(i,:) = (Rx_mat * (Ry_mat * (Rz_mat * Sixgle_pos(i,:)')))';
            Upper_pos(i,:) = (Rx_mat * (Ry_mat * (Rz_mat * Upper_pos(i,:)')))';
        end
        Center_pos0 = (Rx_mat * (Ry_mat * (Rz_mat * Center_pos0')))';
%% 计算杆的干涉
        tic
        Infer_val = calbarinferfuc(Infer_val, Center_pos0, (0.9 * Center_pos0 + 0.1 * Rotate_Center), r + 100);
        for kk = 1:6
            Infer_val = calbarinferfuc(Infer_val, Sixgle_pos(kk,:), Base_pos(kk,:), 100);  %% 杆的半径估算
        end
        toc
    
    end
    
end