function [Opt_index, Upper_mov, Low_mov] = calmovingrange(R,r,D,d,Base_H,Center_H,Upper_Radis,UpperJ_Radis,Mid,Thres)
%   在R,r,D,d参数下形成最高点和最低点
%   R建议为2157.1/2, r建议为1209.70/2, D建议为300/2, d建议为250/2
%   R为下面盘的半径, r是上面盘的半径, D为下面盘的弦长, d是上面盘的弦长, Mid为中位位置
    Upper_mov = 0;
    Low_mov = inf;
    Rotate_Center = [0,0,0];
    Pitch_Amp = 20;
    Yaw_Amp = 20;
    Rotate_Amp = 30;
    X_Amp = 200;
    Y_Amp = 200;
    Z_Amp = 200;
%% 计算正则项
    min_distance = inf;
    Pitch_rec = [[-Pitch_Amp:0.1*Pitch_Amp:Pitch_Amp],zeros(1,84)];
    Yaw_rec = [zeros(1,21),[-Yaw_Amp:0.1*Yaw_Amp:Yaw_Amp],zeros(1,63)];
    Rotate_rec = [zeros(1,42),[-Rotate_Amp:0.1*Rotate_Amp:Rotate_Amp],zeros(1,42)];
    X_rec = [zeros(1,63),[-X_Amp:0.1*X_Amp:X_Amp],zeros(1,21)];
    Y_rec = [zeros(1,84),[-Y_Amp:0.1*Y_Amp:Y_Amp]];
    Z_rec = zeros(1,105);
    for TT = 1:105
        Center_pos = [X_rec(TT) + Rotate_Center(1) - UpperJ_Radis * sind(Yaw_rec(TT)), Y_rec(TT) + Rotate_Center(2) - UpperJ_Radis * cosd(Yaw_rec(TT)) * sind(Pitch_rec(TT)), Z_rec(TT) + Rotate_Center(3) - UpperJ_Radis * cosd(Yaw_rec(TT)) * cosd(Pitch_rec(TT))];
        End_pos = CalresPoint(R,r,D,d,Center_pos,Base_H,Pitch_rec(TT),Yaw_rec(TT),Rotate_rec(TT));
        min_distance = min(min_distance,calculate_min_segment_distance(End_pos));
    end
%% 计算行程
    Pitch_rec = [0,Pitch_Amp,-Pitch_Amp,0,0,0,0,0,0,0,0,0,0];
    Yaw_rec = [0,0,0,Yaw_Amp,-Yaw_Amp,0,0,0,0,0,0,0,0];
    Rotate_rec = [0,0,0,0,0,Rotate_Amp,-Rotate_Amp,0,0,0,0,0,0];
    X_rec = [0,0,0,0,0,0,0,X_Amp,-X_Amp,0,0,0,0];
    Y_rec = [0,0,0,0,0,0,0,0,0,Y_Amp,-Y_Amp,0,0];
    Z_rec = [0,0,0,0,0,0,0,0,0,0,0,Z_Amp,-Z_Amp];
    for TT = 1:13
        Center_pos = [X_rec(TT) + Rotate_Center(1) - UpperJ_Radis * sind(Yaw_rec(TT)), Y_rec(TT) + Rotate_Center(2) - UpperJ_Radis * cosd(Yaw_rec(TT)) * sind(Pitch_rec(TT)), Z_rec(TT) + Rotate_Center(3) - UpperJ_Radis * cosd(Yaw_rec(TT)) * cosd(Pitch_rec(TT))];
        Center_pos0 = [X_rec(TT) + Rotate_Center(1) - Upper_Radis * sind(Yaw_rec(TT)), Y_rec(TT) + Rotate_Center(2) - Upper_Radis * cosd(Yaw_rec(TT)) * sind(Pitch_rec(TT)), Z_rec(TT) + Rotate_Center(3) - Upper_Radis * cosd(Yaw_rec(TT)) * cosd(Pitch_rec(TT))];
        [Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLength(R,r,D,d,Center_pos,Center_pos0,Base_H,Pitch_rec(TT),Yaw_rec(TT),Rotate_rec(TT));  
        for L = 1:6
            Upper_mov = max(Upper_mov, Para_opt(L,1));
            Low_mov = min(Low_mov, Para_opt(L,1));
        end
    end
    Opt_index = max(abs(Upper_mov - Mid), abs(Mid - Low_mov));
    if(min_distance < Thres)
        Opt_index = Opt_index + 1000 * Thres/min_distance;
    end
    if(r < 500)
        Opt_index = Opt_index + 1000 * (500/r);
    end
end

