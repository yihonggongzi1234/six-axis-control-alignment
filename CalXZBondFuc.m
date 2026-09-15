R = 1078.55;
r = 604.85;
D = 150;
d = 125;
Base_H = 253;
Center_H = 2700;
Upper_Radis = 899.2;
UpperJ_Radis = 1080.2;

XZ_mat = zeros(521,2);
XZ_mat(:,2) = [-260:1:260]';
for j = 1:521
    k = 0;
    Max_Lenrec = 0;
    Min_Lenrec = 2000;
    while((Max_Lenrec < 1820.0) && (Min_Lenrec >= 1300.0))
        k = k + 0.1;
        [Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLength(R,r,D,d,Base_H,Center_H,Upper_Radis,UpperJ_Radis,0,0,0,k,0,XZ_mat(j,2));
        Max_Lenrec = max(Para_opt(:,1));
        Min_Lenrec = min(Para_opt(:,1));
    end
    XZ_mat(j,1) = k - 0.1;
end