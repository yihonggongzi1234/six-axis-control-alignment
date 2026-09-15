function [Error_vec] = showerroraboutbar(R,r,D,d,Base_H,Center_H,UpperJ_Radis,L)
%   记录引发误差带来的杆长差距
%   本记录基于反解体系完成
    dif_recmat = zeros(64,6);
    [Angle_RXC, Angle_RYC, Angle_RZC, X_MovC, Y_MovC, Z_MovC, ~, ~] = solveSixAxisPlane(R,r,D,d,Base_H,Center_H,UpperJ_Radis,0,L);
    N_id = 0;
    for i1 = 1:2
        for i2 = 1:2
            for i3 = 1:2
                for i4 = 1:2
                    for i5 = 1:2
                        for i6 = 1:2
                            L_ipt = [L(1)+(i1 - 1.5) * 0.2,L(2)+(i2 - 1.5) * 0.2,L(3)+(i3 - 1.5) * 0.2,L(4)+(i4 - 1.5) * 0.2,L(5)+(i5 - 1.5) * 0.2,L(6)+(i6 - 1.5) * 0.2];
                            N_id = N_id + 1;
                            [Angle_RX, Angle_RY, Angle_RZ, X_Mov, Y_Mov, Z_Mov, ~, ~] = solveSixAxisPlane(R,r,D,d,Base_H,Center_H,UpperJ_Radis,0,L_ipt);
                            dif_recmat(N_id,:) = [Angle_RX - Angle_RXC,Angle_RY - Angle_RYC,Angle_RZ - Angle_RZC,X_Mov - X_MovC,Y_Mov - Y_MovC,Z_Mov - Z_MovC];
                        end
                    end
                end
            end
        end
    end
    Error_vec = max(abs(dif_recmat));
end

