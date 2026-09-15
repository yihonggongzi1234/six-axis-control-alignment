function [Center_pos,Angle_Pitch,Angle_Yaw,Tangle_pos,Para_opt] = Cal6motion(R,r,Ipt_para)
% 基本假设，六杆的基点位置为0/60/120/180/240/300和360度
% 输入参数为杠基距离盘中心的半径R, 杆顶距离平台中心的半径r   
% 基座中心坐标为(0,0), 主动杆为0度，120度和240度
% 输出的Angle_Pitch为俯仰角, Angle_Yaw为偏航角, Center_pos为重心的(X,Y,Z)坐标
% Tangle_pos为三角平台的三个顶点坐标
% Para_opt对于杆信息的输出格式分别为(第一行杠长度, 第二行杠在水平面角度，第三行在铅垂面角度)
% 输入六个自由度(Ipt_para),分别为0度杆的长度,水平角度,垂直角度,120度杆的水平角度,垂直角度,240度杆的长度
    Para_opt = NaN * ones(3,6);
    Para_opt(1,1) = Ipt_para(1);   % 1号杆的长度
    Para_opt(1,2) = Ipt_para(2)*pi()/180;   % 1号杆在水平方向的角度
    Para_opt(1,3) = Ipt_para(3)*pi()/180;   % 1号杆在竖直方向的长度
    Para_opt(3,2) = Ipt_para(4)*pi()/180;   % 3号杆在水平方向的角度
    Para_opt(3,3) = Ipt_para(5)*pi()/180;   % 3号杆在竖直方向的角度
    Para_opt(5,2) = Ipt_para(6)*pi()/180;   % 5号杆的在水平方向的角度
    Base_pos = NaN * ones(6,3);
    Tangle_pos = NaN * ones(3,3);
    %% 首先计算第一个三角形点的坐标
    for i = 0:5
        Base_pos(i+1,1) = R * cos(i*2*pi()/6);
        Base_pos(i+1,2) = R * sin(i*2*pi()/6);
        Base_pos(i+1,3) = 0;
    end
    Tangle_pos(1,1) = Base_pos(1,1) + Para_opt(1,1) * cos(Para_opt(1,3)) * cos(Para_opt(1,2));
    Tangle_pos(1,2) = Base_pos(1,2) + Para_opt(1,2) * cos(Para_opt(1,3)) * sin(Para_opt(1,2));
    Tangle_pos(1,3) = Base_pos(1,3) + Para_opt(1,3) * sin(Para_opt(1,3));
    %% 计算第二个三角形点的坐标
    k31 = cos(Para_opt(3,3)) * cos(Para_opt(3,2));
    k32 = cos(Para_opt(3,3)) * sin(Para_opt(3,2));
    k33 = sin(Para_opt(3,3));
    R3 = k31 * (Tangle_pos(1,1) - Base_pos(3,1)) + k32 * (Tangle_pos(1,2) - Base_pos(3,2)) + k33 * (Tangle_pos(1,3) - Base_pos(3,3));
    Tmp_key3(1) = Base_pos(3,1) + k31 * R3;
    Tmp_key3(2) = Base_pos(3,2) + k32 * R3;
    Tmp_key3(3) = Base_pos(3,3) + k33 + R3;
    Dist_tmp = sqrt((Tangle_pos(1,1) - Tmp_key3(1))^2 + (Tangle_pos(1,2) - Tmp_key3(2))^2 + (Tangle_pos(1,3) - Tmp_key3(3))^2);
    if(Dist_tmp > sqrt(3)*r)
        error('错误输入');
    else
        R3 = R3 - sqrt(3*r^2 - Disp_tmp^2);
    end
    Tangle_pos(2,1) = Base_pos(3,1) + k31 * R3;
    Tangle_pos(2,2) = Base_pos(3,2) + k32 * R3;
    Tangle_pos(2,3) = Base_pos(3,3) + k33 + R3;
    %% 计算最后一个三角点的坐标
    for j = 1:3
        Tmp_Cent(j) = (Tangle_pos(1,j) + Tangle_pos(2,j))/2;
        Vert_K(j) = Tangle_pos(2,j) - Tangle_pos(1,j);
    end
    Vert_Norm = sqrt(Vert_K(1)^2 + Vert_K(2)^2 + Vert_K(3)^2);
    for j = 1:3
        Vert_K(j) = Vert_K(j)/Vert_Norm;
    end
    R_A = 1.5*r;
    R_B = 1.5*r*sqrt(1-Vert_K(3)^2);
    R_Theta = atan(Vert_K(2)/Vert_K(1));
    K_X = cos(Para_opt(5,2));
    K_Y = sin(Para_opt(5,2));
    Para_M = R_A * cos(R_Theta) * K_Y - R_A * sin(R_Theta) * K_X;
    Para_N = - R_B * sin(R_Theta) * K_Y - R_B * cos(R_Theta) * K_X;
    Para_P = (Tmp_Cent(2) - Base_pos(5,2)) * K_X - (Tmp_Cent(1) - Base_pos(5,1)) * K_Y;
    if(Para_P > sqrt(Para_M^2 + Para_N^2))
        error('错误输入,无法生成三角形第三个点');
    else
        Para_CT = R_Theta - acos(Para_P/sqrt(Para_M^2 + Para_N^2));  % 这里有2个解
    end
    Tangle_pos(3,1) = Tmp_Cent(1) + R_A * cos(R_Theta) * cos(Para_CT) - R_B * sin(R_Theta) * sin(Para_CT);
    Tangle_pos(3,2) = Tmp_Cent(2) + R_A * sin(R_Theta) * cos(Para_CT) + R_B * cos(R_Theta) * sin(Para_CT);
    Tangle_pos(3,3) = Tmp_Cent(3) - sqrt(R_A^2 - (Tangle_pos(3,1) - Tmp_Cent(1))^2 - (Tangle_pos(3,2) - Tmp_Cent(2))^2);  % 这里有两个解
    %% 反算六个杆的其他参数
    for j = 1:6
        k = round(j/2 + 0.1);
        Bar_Arrow = Tangle_pos(k,:) - Base_pos(j,:);
        Para_opt(j,1) = sqrt(Bar_Arrow(1)^2 + Bar_Arrow(2)^2 + Bar_Arrow(3)^2);
        if(Bar_Arrow(1) > 0)
            Para_opt(j,2) = atan(Bar_Arrow(2)/Bar_Arrow(1));
        else
            Para_opt(j,2) = atan(Bar_Arrow(2)/Bar_Arrow(1)) + pi();
        end
        Para_opt(j,3) = atan(Bar_Arrow(3)/sqrt(Bar_Arrow(1)^2 + Bar_Arrow(2)^2));
    end
    Para_opt(:,2:3) = Para_opt(:,2:3) * 180/pi();
    %% 计算平面参数
    Plane_Vert(1) = (Tangle_pos(2,2) - Tangle_pos(1,2))*(Tangle_pos(3,3) - Tangle_pos(1,3)) - (Tangle_pos(3,2) - Tangle_pos(1,2))*(Tangle_pos(2,3) - Tangle_pos(1,3));
    Plane_Vert(2) = (Tangle_pos(2,3) - Tangle_pos(1,3))*(Tangle_pos(3,1) - Tangle_pos(1,1)) - (Tangle_pos(3,3) - Tangle_pos(1,3))*(Tangle_pos(2,1) - Tangle_pos(1,1));
    Plane_Vert(3) = (Tangle_pos(2,1) - Tangle_pos(1,1))*(Tangle_pos(3,2) - Tangle_pos(1,2)) - (Tangle_pos(3,1) - Tangle_pos(1,1))*(Tangle_pos(2,2) - Tangle_pos(1,2));
    Plane_Norm = sqrt(Plane_Vert(1)^2 + Plane_Vert(2)^2 + Plane_Vert(3)^2);
    Center_pos = NaN * ones(1,3);
    for i = 1:3
        Plane_Vert(i) = Plane_Vert(i) / Plane_Norm;
        Center_pos(i) = sum(Tangle_pos(:,i))/3;
    end
    Angle_Pitch = 180/pi() * atan(sqrt(Plane_Vert(1)^2 + Plane_Vert(2)^2)/Plane_Vert(3));
    if(Plane_Vert(1) >= 0)
        Angle_Yaw = 180/pi() * atan(Plane_Vert(2)/Plane_Vert(1));
    else
        Angle_Yaw = 180/pi() * (mod(pi() + atan(Plane_Vert(2)/Plane_Vert(1)),2*pi()));
    end
end

