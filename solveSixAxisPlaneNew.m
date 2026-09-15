function [Angle_RX, Angle_RY, Angle_RZ, X_Mov, Y_Mov, Z_Mov, fval] = solveSixAxisPlaneNew(R,r,D,d,Base_H,Center_H,UpperJ_Radis,Cen_X,Cen_Y,is_move,L,is_guess,para_guess)
    % R:下面盘的半径; r:上面盘的半径; D:下面盘的弦长; d:上面盘的弦长; Upper2gd:动平台静位距地;
    % Upper_Radis为上平台的回转半径; Lower2bt: 静铰点距台底面;Upper2ht: 动铰点距动平面; H_iron: 垫铁高度;
    % Cen_X回转中心在X方向的平移; Cen_Y回转中心在Y方向的平移
    % P: 6x3 matrix, base points
    % L: 6x1 vector, lengths
    Base_H = Lower2bt + H_iron;     % Base_H:下面台子的高度
    Center_H = Upper2gd + Upper_Radis;    % Center_H: 回转中心的高度
    UpperJ_Radis = Upper_Radis + Upper2ht;   % UpperJ_Radis为上铰链的回转半径
    %% 后续要把Center_pos改成X Y Z三个方向的MOV以及进行验算以验证结果有效。
    % 粒子群参数
    opts.step_scale = 0.3;
    if(is_guess > 0)
        opts.max_iter = 50;       % 最大迭代次数
        opts.pop_size = 50;       % 群体数量
        opts.lb = [para_guess(1) - 0.2,para_guess(2) - 0.2,para_guess(3) - 0.2,para_guess(4) - 2,para_guess(5) - 2,para_guess(6) - 2];
        opts.ub = [para_guess(1) + 0.2,para_guess(2) + 0.2,para_guess(3) + 0.2,para_guess(4) + 2,para_guess(5) + 2,para_guess(6) + 2];
        is_anglemove = max(abs(sign(para_guess(1:3))));
        is_sixmove = max(abs(sign(para_guess(1:6))));
        for dima_id = 1:3
            if(para_guess(dima_id) == 0)
                if((is_anglemove == 0) && (is_sixmove > 0))
                    opts.lb(1,dima_id) = 0;
                    opts.ub(1,dima_id) = 0;
                else
                    opts.lb(1,dima_id) = -0.2;
                    opts.ub(1,dima_id) = 0.2;
                end
            end
        end
        for dima_id = 4:6
            if(para_guess(dima_id) == 0) 
                if(((is_anglemove == 0) && (is_sixmove > 0)) && (dima_id <= 5))
                    opts.lb(1,dima_id) = 0;
                    opts.ub(1,dima_id) = 0;
                else
                    opts.lb(1,dima_id) = -2;
                    opts.ub(1,dima_id) = 2;
                end
            end
        end
    else
        opts.max_iter = 500;       % 最大迭代次数
        opts.pop_size = 50;       % 群体数量
        opts.lb = [-20,-20,-30,-300,-300,-300];
        opts.ub = [20,20,30,300,300,300];
    end
    [angles_opt, fval] = global_optimize(@residuals, para_guess, R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L, opts);
    
    % 输出平面参数
    Angle_RX = angles_opt(1);
    Angle_RY = angles_opt(2);
    Angle_RZ = angles_opt(3);
    X_Mov = angles_opt(4);
    Y_Mov = angles_opt(5);
    Z_Mov = angles_opt(6);
end

function rads = residuals(angles_flat, R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L)
    Angle_RX = angles_flat(1);
    Angle_RY = angles_flat(2);
    Angle_RZ = angles_flat(3);
    X_Mov = angles_flat(4);
    Y_Mov = angles_flat(5);
    Z_Mov = angles_flat(6);    
    
    Rotate_Center = [Cen_X,Cen_Y,0];
    Center_pos = [X_Mov + Rotate_Center(1), Y_Mov + Rotate_Center(2), Z_Mov + Rotate_Center(3) + UpperJ_Radis];
    a_theta = asin(d/(2*r)) * 180 / pi();
    Upper_angle = 0 - [-a_theta,a_theta,120 - a_theta,120 + a_theta,240 - a_theta,240 + a_theta];
    r_Mat = NaN * ones(6,3);
    for i = 1:6
        r_Mat(i,1) = r * cosd(Upper_angle(i));
        r_Mat(i,2) = - r * sind(Upper_angle(i));
        r_Mat(i,3) = 0;
    end
    P_U = NaN * ones(6,3);
    for i = 1:6
        P_U(i,:) = Center_pos + r_Mat(i,:);
    end
%% 进行矩阵旋转
    Rx_mat = [1, 0, 0; 0, cosd(Angle_RX), -sind(Angle_RX); 0, sind(Angle_RX),  cosd(Angle_RX)];
    Ry_mat = [cosd(Angle_RY), 0, sind(Angle_RY); 0, 1, 0; -sind(Angle_RY), 0, cosd(Angle_RY)];
    Rz_mat = [cosd(Angle_RZ),  sind(Angle_RZ), 0; -sind(Angle_RZ),  cosd(Angle_RZ), 0; 0, 0, 1];
    for i = 1:6
        P_U(i,:) = (Rx_mat * (Ry_mat * (Rz_mat * P_U(i,:)')))';
    end
    b_theta = asin(D/(2*R)) * 180 / pi();
    Base_angle = - [b_theta - 60,60 - b_theta,60 + b_theta,180 - b_theta,180 + b_theta, 300-b_theta];
    P_B = NaN * ones(6,3);
    for i = 1:6
        P_B(i,1) = R * cosd(Base_angle(i));
        P_B(i,2) = -R * sind(Base_angle(i));
        P_B(i,3) = Center_H - Base_H;   % 厚度
    end
    P_L = zeros(6,1);
    for i = 1:6
        P_L(i) = sqrt((P_B(i,1) - P_U(i,1))^2 + (P_B(i,2) - P_U(i,2))^2 + (P_B(i,3) - P_U(i,3))^2);
    end
    rads = 0;
    for i = 1:6
        rads = rads + (L(i) - P_L(i))^2;
    end
end

function [best_sol, best_val] = global_optimize(residuals_func, init_guess, R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L, opts)
    % 参数设置
    max_iter = opts.max_iter;       % 最大迭代次数
    pop_size = opts.pop_size;       % 群体数量
    var_dim  = numel(init_guess);   % 变量维度
    lb = opts.lb(:);                % 下界
    ub = opts.ub(:);                % 上界
    step_scale = opts.step_scale;   % 每步扰动范围比例

    % 初始化种群
    population = zeros(pop_size,var_dim + 1);
    population(1:pop_size-1,1:var_dim) = repmat(init_guess(:)', pop_size-1, 1) + ...
                 2*(rand(pop_size-1, var_dim) - 0.5) .* (ub' - lb') * 0.5;
    population(pop_size,1:var_dim) = init_guess;

    % 裁剪到上下限
    population(:,1:var_dim) = min(max(population(:,1:var_dim), lb'), ub');

    % 计算初始适应度
    for i = 1:pop_size
        population(i,var_dim+1) = residuals_func(population(i,1:var_dim), R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L);
    end

    % 找到初始最优
    [best_val, idx] = min(population(:,var_dim+1));
    best_sol = population(idx, :);

    % 全局迭代
    iter = 0;
    while((iter <= max_iter) && (best_val > 0.01))
        %% 变异模块
        iter = iter + 1;
        Npopulation = zeros(pop_size, var_dim + 1);
        Npopulation(:,1:var_dim) = population(:,1:var_dim) + (rand(pop_size, var_dim) - 0.5) .* (ub' - lb') * max(step_scale * 0.001, step_scale * 10^(-3*iter/max_iter));
        % 裁剪到上下限
        population(:,1:var_dim) = min(max(population(:,1:var_dim), lb'), ub');
        for i = 1:pop_size
            Npopulation(i,var_dim+1) = residuals_func(Npopulation(i,1:var_dim), R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L);
        end
        population = [population;Npopulation];
        [~,sort_id] = sort(population(:,var_dim+1),'ascend');
        population = population(sort_id(1:pop_size),:);
        %% 单独训练模块
        for i = 1:pop_size
            IID = randi([1,var_dim],1,1);
            Spopulation = population(i,:);
            Spopulation(1,IID) = Spopulation(1,IID) + abs(rand(1,1) - 0.5) * max(step_scale * 0.001, step_scale * 10^(-3*iter/max_iter));
            Spopulation(1,var_dim+1) = residuals_func(Spopulation(1,1:var_dim), R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L);
            if(Spopulation(1,var_dim+1) < population(i,var_dim+1))
                population(i,:) = Spopulation;
            end
        end
        %% 交叉模块
        P_R = abs(rand(pop_size, var_dim) - 0.5);
        Npopulation = population;
        if(iter == 100)
            disp('暂停');
        end
        for i = 1:pop_size
            Tmp_i = randi([1,pop_size],1,1);
            Npopulation(i, P_R(i,:) > 0) = population(Tmp_i, P_R(i,:) > 0);
            Npopulation(i,var_dim+1) = residuals_func(Npopulation(i,1:var_dim), R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L);
        end
        population = [population;Npopulation];
        [~,sort_id] = sort(population(:,var_dim+1),'ascend');
        population = population(sort_id(1:pop_size),:);

        % 可视化或调试输出
        [best_val, idx] = min(population(:,var_dim+1));
        best_sol = population(idx, 1:var_dim);
        %fprintf('Iter %d: Best Val = %.6f\n', iter, best_val);
    end
    fprintf('Iter %d: Best Val = %.6f\n', iter, best_val);
end
