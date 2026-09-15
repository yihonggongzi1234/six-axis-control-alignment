function [Length_record,Theta_record1,Theta_record2,Max_pos,Min_pos] = calmovfunc(R,r,D,d,Upper2gd,Upper_Radis,Lower2bt,Upper2ht,H_iron,Cen_X,Cen_Y,RX_Amp,RY_Amp,RZ_Amp,X_Amp,Y_Amp,Z_Amp,Motion_Frequency,Total_Time)
% R:下面盘的半径; r:上面盘的半径; D:下面盘的弦长; d:上面盘的弦长; Upper2gd:动平台静位距地;
% Upper_Radis为上平台的回转半径; Lower2bt: 静铰点距台底面;Upper2ht: 动铰点距动平面; H_iron: 垫铁高度;
% Cen_X回转中心在X方向的平移; Cen_Y回转中心在Y方向的平移
% RX_Amp,RY_Amp,RZ_Amp分别是绕X,Y,Z轴的旋转幅值
% X_Amp,Y_Amp,Z_Amp分别是X,Y,Z三个方向的平移幅值
% Motion_Frequency为运动频率(Hz), Total_Time为仿真总时间(s)

    global Stop_Simulation
    Stop_Simulation = false;

    if nargin < 18 || isempty(Motion_Frequency) || ~isfinite(Motion_Frequency)
        Motion_Frequency = 0.5;
    end
    if nargin < 19 || isempty(Total_Time) || ~isfinite(Total_Time)
        Total_Time = 2.0;
    end
    if Motion_Frequency <= 0
        error('Motion_Frequency must be greater than 0.');
    end
    if Total_Time <= 0
        error('Total_Time must be greater than 0.');
    end

    Base_H = Lower2bt + H_iron;
    Center_H = Upper2gd + Upper_Radis;
    UpperJ_Radis = Upper_Radis + Upper2ht;
    Rotate_Center = [Cen_X,Cen_Y,0];

    % 200 samples per motion cycle preserves the original sampling density.
    Samples_Per_Cycle = 200;
    Sample_Time = 1 / (Motion_Frequency * Samples_Per_Cycle);
    Num_Intervals = max(1, round(Total_Time / Sample_Time));
    Time_vec = linspace(0, Total_Time, Num_Intervals + 1)';
    Phase_deg = 360 * Motion_Frequency * Time_vec;

    RX_rec = sind(Phase_deg) * RX_Amp;
    RY_rec = sind(Phase_deg) * RY_Amp;
    RZ_rec = sind(Phase_deg) * RZ_Amp;
    X_rec = sind(Phase_deg) * X_Amp;
    Y_rec = sind(Phase_deg) * Y_Amp;
    Z_rec = sind(Phase_deg) * Z_Amp;

    Num_Samples = numel(Time_vec);
    Length_record = zeros(Num_Samples,12);
    Length_record(:,1) = RX_rec;
    Length_record(:,2) = RY_rec;
    Length_record(:,3) = RZ_rec;
    Length_record(:,4) = X_rec;
    Length_record(:,5) = Y_rec;
    Length_record(:,6) = Z_rec;

    Theta_record1 = Length_record;
    Theta_record2 = Length_record;
    Max_pos = -10000 * ones(6,3);
    Min_pos = 10000 * ones(6,3);
    Completed_Samples = Num_Samples;

    for k = 1:Num_Samples
        if Stop_Simulation
            Completed_Samples = k - 1;
            break;
        end

        Center_pos = [X_rec(k) + Rotate_Center(1), Y_rec(k) + Rotate_Center(2) - UpperJ_Radis, Z_rec(k) + Rotate_Center(3)];
        Center_pos0 = [X_rec(k) + Rotate_Center(1), Y_rec(k) + Rotate_Center(2) - Upper_Radis, Z_rec(k) + Rotate_Center(3)];
        [Para_opt,~,Sixgle_pos] = Calresvmotion(R,r,D,d,Center_pos,Center_pos0,Base_H,Center_H,RX_rec(k),RY_rec(k),RZ_rec(k),ones(6,1),1000);
        for L = 1:6
            Length_record(k,6+L) = Para_opt(L,1);
            Theta_record1(k,6+L) = Para_opt(L,2);
            Theta_record2(k,6+L) = Para_opt(L,3);
            for K = 1:3
                Max_pos(L,K) = max(Max_pos(L,K), Sixgle_pos(L,K));
                Min_pos(L,K) = min(Min_pos(L,K), Sixgle_pos(L,K));
            end
        end
        drawnow;
    end

    if Completed_Samples < Num_Samples
        Length_record = Length_record(1:Completed_Samples,:);
        Theta_record1 = Theta_record1(1:Completed_Samples,:);
        Theta_record2 = Theta_record2(1:Completed_Samples,:);
    end
end