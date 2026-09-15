function [Angle_RX, Angle_RY, Angle_RZ, X_Mov, Y_Mov, Z_Mov, Center_pos, Sixgle_pos] = solveSixAxisPlane(R,r,D,d,Upper2gd,Upper_Radis,Lower2bt,Upper2ht,H_iron,Cen_X,Cen_Y,is_move,L)
    % R:�����̵İ뾶; r:�����̵İ뾶; D:�����̵��ҳ�; d:�����̵��ҳ�; Upper2gd:��ƽ̨��λ���;
    % Upper_RadisΪ��ƽ̨�Ļ�ת�뾶; Lower2bt: ���µ��̨����;Upper2ht: ���µ�දƽ��; H_iron: �����߶�;
    % Cen_X��ת������X�����ƽ��; Cen_Y��ת������Y�����ƽ��
    % P: 6x3 matrix, base points
    % L: 6x1 vector, lengths
    Base_H = Lower2bt + H_iron;     % Base_H:����̨�ӵĸ߶�
    Center_H = Upper2gd + Upper_Radis;    % Center_H: ��ת���ĵĸ߶�
    UpperJ_Radis = Upper_Radis + Upper2ht;   % UpperJ_RadisΪ�Ͻ����Ļ�ת�뾶
    %% ����Ҫ��Center_pos�ĳ�X Y Z���������MOV�Լ�������������֤�����Ч��
    % ����Ⱥ����
    opts.step_scale = 0.3;
    opts.max_iter = 500;       % ����������
    opts.pop_size = 50;       % Ⱥ������
    if(is_move > 0)
        opts.lb = [-20,-20,-30,-300,-300,-300];
        opts.ub = [20,20,30,300,300,300];
    else
        opts.lb = [-20,-20,-30,0,0,0];
        opts.ub = [20,20,30,0,0,0];
    end
    angles0 = [0,0,0,0,0,0];
    [angles_opt, fval] = global_optimize(@residuals, angles0, R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L, opts);
    
    % ���ƽ�����
    Angle_RX = angles_opt(1);
    Angle_RY = angles_opt(2);
    Angle_RZ = angles_opt(3);
    X_Mov = angles_opt(4);
    Y_Mov = angles_opt(5);
    Z_Mov = angles_opt(6);
    
    
    Rotate_Center = [Cen_X,Cen_Y,0];
    Center_pos = [X_Mov + Rotate_Center(1), Y_Mov + Rotate_Center(2) - UpperJ_Radis, Z_Mov + Rotate_Center(3)];
    a_theta = asin(d/r) * 180 / pi();
    Upper_angle = 0 - [-a_theta,a_theta,120 - a_theta,120 + a_theta,240 - a_theta,240 + a_theta];
    r_Mat = NaN * ones(6,3);
    for i = 1:6
        r_Mat(i,1) = r * cosd(Upper_angle(i));
        r_Mat(i,2) = 0;
        r_Mat(i,3) = r * sind(Upper_angle(i));
    end
    Sixgle_pos = NaN * ones(6,3);
    for i = 1:6
        Sixgle_pos(i,:) = Center_pos + r_Mat(i,:);
    end
%% ���о�����ת
    Rx_mat = [1, 0, 0; 0, cosd(Angle_RX), -sind(Angle_RX); 0, sind(Angle_RX),  cosd(Angle_RX)];
    Ry_mat = [cosd(Angle_RY), 0, sind(Angle_RY); 0, 1, 0; -sind(Angle_RY), 0, cosd(Angle_RY)];
    Rz_mat = [cosd(Angle_RZ), -sind(Angle_RZ), 0; sind(Angle_RZ), cosd(Angle_RZ), 0; 0, 0, 1];
    Rotation_mat = Rx_mat * Ry_mat * Rz_mat;
    for i = 1:6
        Sixgle_pos(i,:) = Center_pos + (Rotation_mat * r_Mat(i,:)')';
    end
    Center_pos(1) = mean(Sixgle_pos(:,1));
    Center_pos(2) = mean(Sixgle_pos(:,2));
    Center_pos(3) = mean(Sixgle_pos(:,3));    
end

function rads = residuals(angles_flat, R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L)
    Angle_RX = angles_flat(1);
    Angle_RY = angles_flat(2);
    Angle_RZ = angles_flat(3);
    X_Mov = angles_flat(4);
    Y_Mov = angles_flat(5);
    Z_Mov = angles_flat(6);   
    
    Rotate_Center = [Cen_X,Cen_Y,0];
    Center_pos = [X_Mov + Rotate_Center(1), Y_Mov + Rotate_Center(2)- UpperJ_Radis, Z_Mov + Rotate_Center(3)];
    a_theta = asin(d/(2*r)) * 180 / pi();
    Upper_angle = 0 - [-a_theta,a_theta,120 - a_theta,120 + a_theta,240 - a_theta,240 + a_theta];
    r_Mat = NaN * ones(6,3);
    for i = 1:6
        r_Mat(i,1) = r * cosd(Upper_angle(i));
        r_Mat(i,2) = 0 ;
        r_Mat(i,3) = r * sind(Upper_angle(i));
    end
    P_U = NaN * ones(6,3);
    for i = 1:6
        P_U(i,:) = Center_pos + r_Mat(i,:);
    end
%% ���о�����ת
    Rx_mat = [1, 0, 0; 0, cosd(Angle_RX), -sind(Angle_RX); 0, sind(Angle_RX),  cosd(Angle_RX)];
    Ry_mat = [cosd(Angle_RY), 0, sind(Angle_RY); 0, 1, 0; -sind(Angle_RY), 0, cosd(Angle_RY)];
    Rz_mat = [cosd(Angle_RZ), -sind(Angle_RZ), 0; sind(Angle_RZ), cosd(Angle_RZ), 0; 0, 0, 1];
    Rotation_mat = Rx_mat * Ry_mat * Rz_mat;
    for i = 1:6
        P_U(i,:) = Center_pos + (Rotation_mat * r_Mat(i,:)')';
    end
    b_theta = asin(D/(2*R)) * 180 / pi();
    Base_angle = - [b_theta - 60,60 - b_theta,60 + b_theta,180 - b_theta,180 + b_theta, 300-b_theta];
    P_B = NaN * ones(6,3);
    for i = 1:6
        P_B(i,1) = R * cosd(Base_angle(i));
        P_B(i,2) = -(Center_H - Base_H);
        P_B(i,3) = R * sind(Base_angle(i));   % ���
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
    % ��������
    max_iter = opts.max_iter;       % ����������
    pop_size = opts.pop_size;       % Ⱥ������
    var_dim  = numel(init_guess);   % ����ά��
    lb = opts.lb(:);                % �½�
    ub = opts.ub(:);                % �Ͻ�
    step_scale = opts.step_scale;   % ÿ���Ŷ���Χ����

    % ��ʼ����Ⱥ
    population = zeros(pop_size,var_dim + 1);
    population(1:pop_size-1,1:var_dim) = repmat(init_guess(:)', pop_size-1, 1) + ...
                 2*(rand(pop_size-1, var_dim) - 0.5) .* (ub' - lb') * 0.5;
    population(pop_size,1:var_dim) = init_guess;

    % �ü���������
    population(:,1:var_dim) = min(max(population(:,1:var_dim), lb'), ub');

    % �����ʼ��Ӧ��
    for i = 1:pop_size
        population(i,var_dim+1) = residuals_func(population(i,1:var_dim), R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L);
    end

    % �ҵ���ʼ����
    [best_val, idx] = min(population(:,var_dim+1));
    best_sol = population(idx, :);

    % ȫ�ֵ���
    iter = 0;
    while((iter <= max_iter) && (best_val > 0.01))
        %% ����ģ��
        iter = iter + 1;
        Npopulation = zeros(pop_size, var_dim + 1);
        Npopulation(:,1:var_dim) = population(:,1:var_dim) + (rand(pop_size, var_dim) - 0.5) .* (ub' - lb') * max(step_scale * 0.001, step_scale * 10^(-3*iter/max_iter));
        % �ü���������
        population(:,1:var_dim) = min(max(population(:,1:var_dim), lb'), ub');
        for i = 1:pop_size
            Npopulation(i,var_dim+1) = residuals_func(Npopulation(i,1:var_dim), R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L);
        end
        population = [population;Npopulation];
        [~,sort_id] = sort(population(:,var_dim+1),'ascend');
        population = population(sort_id(1:pop_size),:);
        %% ����ѵ��ģ��
        for i = 1:pop_size
            IID = randi([1,var_dim],1,1);
            Spopulation = population(i,:);
            Spopulation(1,IID) = Spopulation(1,IID) + abs(rand(1,1) - 0.5) * max(step_scale * 0.001, step_scale * 10^(-3*iter/max_iter));
            Spopulation(1,var_dim+1) = residuals_func(Spopulation(1,1:var_dim), R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L);
            if(Spopulation(1,var_dim+1) < population(i,var_dim+1))
                population(i,:) = Spopulation;
            end
        end
        %% ����ģ��
        P_R = abs(rand(pop_size, var_dim) - 0.5);
        Npopulation = population;
        if(iter == 100)
            disp('��ͣ');
        end
        for i = 1:pop_size
            Tmp_i = randi([1,pop_size],1,1);
            Npopulation(i, P_R(i,:) > 0) = population(Tmp_i, P_R(i,:) > 0);
            Npopulation(i,var_dim+1) = residuals_func(Npopulation(i,1:var_dim), R, r, D, d, Base_H, Center_H, UpperJ_Radis, Cen_X, Cen_Y, L);
        end
        population = [population;Npopulation];
        [~,sort_id] = sort(population(:,var_dim+1),'ascend');
        population = population(sort_id(1:pop_size),:);

        % ���ӻ���������
        [best_val, idx] = min(population(:,var_dim+1));
        best_sol = population(idx, 1:var_dim);
        %fprintf('Iter %d: Best Val = %.6f\n', iter, best_val);
    end
    fprintf('Iter %d: Best Val = %.6f\n', iter, best_val);
end
