function rads = residuals0(angles_flat, R, r, D, d, Base_H, Center_H, UpperJ_Radis, L)
    Angle_Pitch = angles_flat(1);
    Angle_Yaw = angles_flat(2);
    Angle_rotate = angles_flat(3);
    X_Mov = angles_flat(4);
    Y_Mov = angles_flat(5);
    Z_Mov = angles_flat(6);
    Rotate_Center = [0,0,Center_H];
    Center_pos = [X_Mov + Rotate_Center(1) - UpperJ_Radis * sind(Angle_Yaw), Y_Mov + Rotate_Center(2) - UpperJ_Radis * cosd(Angle_Yaw) * sind(Angle_Pitch), Z_Mov + Rotate_Center(3) - UpperJ_Radis * cosd(Angle_Yaw) * cosd(Angle_Pitch)];
    
    b_theta = asin(D/R) * 180 / pi();
    Base_angle = [b_theta,120 - b_theta,120 + b_theta,240 - b_theta,240 + b_theta, -b_theta];
    P_B = NaN * ones(6,3);
    for i = 1:6
        P_B(i,1) = R * cosd(Base_angle(i));
        P_B(i,2) = R * sind(Base_angle(i));
        P_B(i,3) = Base_H;   % ºñ¶È
    end
    
    a_theta = asin(d/r) * 180 / pi();
    Upper_angle = Angle_rotate + [60-a_theta,60 + a_theta,180 - a_theta,180 + a_theta,300 - a_theta,300 + a_theta];
    r_Mat = NaN * ones(6,2);
    for i = 1:6
        r_Mat(i,1) = r * cosd(Upper_angle(i));
        r_Mat(i,2) = r * sind(Upper_angle(i));
    end
    r_Mat = r_Mat';
    
    Norm_vert(1) = sind(Angle_Yaw);
    Norm_vert(2) = cosd(Angle_Yaw) * sind(Angle_Pitch);
    Norm_vert(3) = cosd(Angle_Yaw) * cosd(Angle_Pitch);
    S_1 = [Norm_vert(3)/sqrt(Norm_vert(1)^2 + Norm_vert(3)^2),0,-Norm_vert(1)/sqrt(Norm_vert(1)^2 + Norm_vert(3)^2)];
    S_2 = cross(Norm_vert,S_1);
    S_2 = S_2/sqrt(S_2(1)^2 + S_2(2)^2 + S_2(3)^2);
    P_U = NaN * ones(6,3);
    for i = 1:6
        P_U(i,:) = Center_pos + r_Mat(1,i) * S_1 + r_Mat(2,i) * S_2;
    end
    P_L = zeros(6,1);
    for i = 1:6
        P_L(i) = sqrt((P_B(i,1) - P_U(i,1))^2 + (P_B(i,2) - P_U(i,2))^2 + (P_B(i,3) - P_U(i,3))^2);
    end
    P_B
    P_U
    P_L
    rads = 0;
    for i = 1:6
        rads = rads + (L(i) - P_L(i))^2;
    end
end