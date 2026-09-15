function gui_calmov_with_plot
    % 主窗口
    fig = figure('Name', '六杠控制算法 反解', 'Position', [50 50 1450 700]);
    global Max_Length Max_Velocity Max_Accel Max_pos Min_pos Stop_Simulation;
    Max_Length = zeros(6,2);
    Max_Velocity = zeros(6, 2);  % 第2维：1为拉伸（正），2为压缩（负）
    Max_Accel = zeros(6, 2);
    Max_pos = -10000 * ones(6,3);
    Min_pos = 10000 * ones(6,3);
    Stop_Simulation = false;
    %% 模块1：系统结构参数
    panel1 = uipanel(fig, 'Title', '系统结构参数', 'FontSize', 12, ...
                     'Position', [0.025 0.71 0.5 0.26]);

    structLabels = {'静铰点半径(R)', '动铰点半径(r)', '静铰点间距(D)', '动铰点间距(d)','动平台静位距地(Upper2gd)', ...
                    ['回转中心距动台','(Upper_Radis)'],['静铰点距台底面','(Lower2bt)'], ['动铰点距动平面','(Upper2ht)'], ['垫铁高度','(H_iron)'], ['中心X偏移','(Cen_X)'], ['中心Y偏移','(Cen_Y)']};
    % 默认参数
    defaultStruct = [1420.2/2, 450.8/2, 180, 135, 748.6, 217, 64.5, 70.5, 0, 0, 0];
    inputs = gobjects(19,1);

    for i = 1:11
        if(i <= 5)
            col = i - 1;
            row = 0;
            xpos = 0.03 + col*0.185;
        else
            col = i - 6;
            row = 1;
            xpos = 0.02 + col*0.155;
        end
        ypos = 1 - row*0.45 - 0.50;
        if(i < 6)
            uicontrol(panel1, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [xpos ypos+0.20 0.2 0.2], 'String', structLabels{i}, ...
            'HorizontalAlignment','left', 'FontSize', 10);
        else
            uicontrol(panel1, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [xpos ypos+0.20 0.2 0.2], 'String', structLabels{i}, ...
            'HorizontalAlignment','left', 'FontSize', 10);
        end
        inputs(i) = uicontrol(panel1, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [xpos ypos 0.12 0.18], 'String', num2str(defaultStruct(i)));
    end
    %% 模块2：运动测试参数
    panel2 = uipanel(fig, 'Title', '运动测试参数', 'FontSize', 12, ...
                     'Position', [0.025 0.45 0.5 0.24]);

    motionLabels = {'Roll amplitude (Rx_Amp)', 'Yaw amplitude (Ry_Amp)', 'Pitch amplitude (Rz_Amp)', ...
                    'X amplitude (X_Amp)', 'Y amplitude (Y_Amp)', 'Z amplitude (Z_Amp)', 'Frequency f (Hz)', 'Motion time T (s)'};
    defaultMotion = [0, 0, 0, 0, 0, 0, 0.5, 2.0];

    for i = 12:19
        col = mod(i - 12, 4);        % 每行最多4列
        row = floor((i - 12) / 4);   % 行数：0 或 1
        xpos = 0.05 + col * 0.23;   % 缩小列间距以适配4列
        ypos = 0.5 - row * 0.45;     % 行间距保持不变

        uicontrol(panel2, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [xpos ypos + 0.2 0.22 0.2], 'String', motionLabels{i - 11}, ...
            'HorizontalAlignment', 'left', 'FontSize', 10);

        inputs(i) = uicontrol(panel2, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [xpos ypos 0.18 0.2], 'String', num2str(defaultMotion(i - 11)));
    end

    %% 模块3：正反解参数
    panel3 = uipanel(fig, 'Title', '反解单元', 'FontSize', 12, ...
                 'Position', [0.025 0.10 0.5 0.33]);  % 控高改小一点

    poseLabels = {'Roll (Angle_RX)', 'Yaw (Angle_RY)', 'Pitch (Angle_RZ)', ...
                  'X offset (X_Mov)', 'Y offset (Y_Mov)', 'Z offset (Z_Mov)'};
    lengthLabels = {'1杆长(L[1])','2杆长(L[2])','3杆长(L[3])','4杆长(L[4])','5杆长(L[5])','6杆长(L[6])'};

    poseInputs = gobjects(6,1);
    lengthInputs = gobjects(6,1);

% 布局参数
    x_start = 0.02;
    x_spacing = 0.155;
    edit_width = 0.13;
    edit_height = 0.16;

% 第一行：姿态输入
    for i = 1:6
        xpos = x_start + (i-1)*x_spacing;
        uicontrol(panel3, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [xpos 0.775 edit_width 0.15], ...
            'String', poseLabels{i}, 'HorizontalAlignment','left', 'FontSize', 10);
        poseInputs(i) = uicontrol(panel3, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [xpos 0.625 edit_width edit_height], 'String', '0');
    end

% 第二行：杆长输入
    for i = 1:6
        xpos = x_start + (i-1)*x_spacing;
        uicontrol(panel3, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [xpos 0.45 edit_width 0.15], ...
            'String', lengthLabels{i}, 'HorizontalAlignment','left', 'FontSize', 10);
        lengthInputs(i) = uicontrol(panel3, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [xpos 0.25 edit_width edit_height], 'String', '0');
    end

% 第三行：复选框，居中
    centerMoveCheckbox = uicontrol(panel3, 'Style', 'checkbox', ...
        'Units', 'normalized', 'Position', [0.4 0.05 0.2 0.15], ...
        'String', '是否控制中心移动', 'FontSize', 10, 'Value', 1);

%     %% 模块4：输出运行正反解文本报告
%     panel4 = uipanel(fig, 'Title', '干涉演示报告', 'FontSize', 12, ...
%                      'Position', [0.55 0.50 0.425 0.45]);
%     myAxes0 = axes('Parent', panel4, 'Position', [0.15 0.10 0.80 0.80]);

%% 模块4：输出6铰点结构示意图
panel4 = uipanel(fig, ...
    'Title', '6铰点结构示意图', 'FontSize', 12, ...
    'Position', [0.55 0.50 0.425 0.45]);
myAxes0 = axes(...
    'Parent', panel4, ...
    'Position', [0.05 0.05 0.9 0.9]);

draw_mechanism_coordinate_sketch(myAxes0);
%% 模块5：输出目前结构尺寸示意图
    panel5 = uipanel(fig, 'Title', '结构示意图', 'FontSize', 12, ...
                     'Position', [0.55 0.05 0.425 0.40]);
    myAxes = axes('Parent', panel5, 'Units', 'normalized', 'Position', [0.05 0.05 0.90 0.90]);
    set(myAxes,'XTick',[],'YTick',[]);                 % 去掉刻度
      % 如还想去掉轴线颜色（基本等同看不到轴）
    set(myAxes,'XColor','none','YColor','none');
    img = imread('929.png');
    imshow(img,'Parent',myAxes,'Border','tight');
    
    %% 底部按钮区域
    btn_names = {'运行', '绘制曲线', '运动报告', '反解', '干涉图解', '停止计算'};
    callbacks = {@run_callback, @plot_callback, @motion_report_callback, ...
                @inverse_kinematics_callback, @calcu_interference, @stop_callback};
    
    for i = 1:length(btn_names)
        btn = uicontrol(fig, 'Style', 'pushbutton', 'String', btn_names{i}, ...
              'FontSize', 11, 'Position', [50 + 120*(i-1), 30, 100, 30], ...
              'Callback', callbacks{i});
        if strcmp(btn_names{i}, '停止计算')
            set(btn, 'BackgroundColor', [1.0 0.75 0.75], 'FontWeight', 'bold');
        end
    end


    %% 数据区
    Length_record = []; Theta_record1 = []; Theta_record2 = [];

    %% ------------------ 回调函数 ------------------
    function run_callback(~, ~)
        
    % 获取输入值，包括频率和运动时间
        values = getValues(inputs);
        f = values{18};
        Total_Time = values{19};
        if ~isfinite(f) || f <= 0 || ~isfinite(Total_Time) || Total_Time <= 0
            errordlg('频率和运动时间必须大于 0。', '参数错误');
            return;
        end
        Stop_Simulation = false;
        [Length_record, Theta_record1, Theta_record2, Max_pos,Min_pos] = calmovfunc(values{1:17}, f, Total_Time);
        if Stop_Simulation
            fprintf('仿真已停止，保留 %d 帧。\n', size(Length_record,1));
            assignin('base', 'Length_record', Length_record);
            assignin('base', 'Theta_record1', Theta_record1);
            assignin('base', 'Theta_record2', Theta_record2);
            return;
        end
        if isempty(Length_record)
            disp('仿真未生成数据。');
            return;
        end
    
        assignin('base', 'Length_record', Length_record);
        assignin('base', 'Theta_record1', Theta_record1);
        assignin('base', 'Theta_record2', Theta_record2);

    % 保存数据
        fid = fopen('RecLen.txt', 'w');
        if fid == -1
            error('无法创建或写入 RecLen.txt');
        end
        for i = 1:size(Length_record, 1)
            fprintf(fid, '%.6f %.6f %.6f %.6f %.6f %.6f\n', Length_record(i,7:12));
        end
        fclose(fid);
        disp('写入 RecLen.txt 完成');

    % 计算速度与加速度
        N = size(Length_record, 1);
        dt = Total_Time / max(N - 1, 1);
        L = Length_record(:,7:12);  % 只取六个长度值

    % 速度与加速度计算
        V = diff(L) / dt;                   % (N-1) × 6
        A = diff(V) / dt;                   % (N-2) × 6
    
    % 绘制图像
        figure()
        hold on
        title('杆的行程曲线')
        for L_id = 1:6
            plot([0:size(L,1)-1]' * dt, L(:,L_id),'linewidth',3);
        end
        xlabel('秒')
        ylabel('毫米')
        figure()
        hold on
        title('杆的速度曲线')
        for L_id = 1:6
            plot([0:size(V,1)-1]' * dt, V(:,L_id),'linewidth',3);
        end
        xlabel('秒')
        ylabel('毫米/秒')
        figure()
        hold on
        title('杆的加速度曲线')
        for L_id = 1:6
            plot([0:size(A,1)-1]' * dt, A(:,L_id),'linewidth',3);
        end
        xlabel('秒')
        ylabel('毫米/秒方')
    % 初始化最大速度、加速度数组（拉伸与压缩）

        for j = 1:6
            vj = V(:,j);
            aj = A(:,j);
            Max_Velocity(j,1) = max(Max_Velocity(j,1),max([0; vj(vj > 0)]));  % 拉伸最大速度
            Max_Velocity(j,2) = min(Max_Velocity(j,2),min([0; vj(vj < 0)]));  % 压缩最大速度
            Max_Accel(j,1)   = max(Max_Accel(j,1),max([0; aj(aj > 0)]));   % 拉伸最大加速度
            Max_Accel(j,2)   = min(Max_Accel(j,2),min([0; aj(aj < 0)]));   % 压缩最大加速度
            Max_Length(j,1) = max(Max_Length(j,1),max(L(:,j)));
            Max_Length(j,2) = min(Max_Length(j,1),min(L(:,j)));
        end
        
        assignin('base', 'Max_Length', Max_Length);
        assignin('base', 'Max_Velocity', Max_Velocity);
        assignin('base', 'Max_Accel', Max_Accel);
        disp('速度与加速度计算完成');
    end

    function stop_callback(~, ~)
        Stop_Simulation = true;
        drawnow;
    end
    
    function calcu_interference(~,~)
    %% 计算系统的干涉情况
    %% 基于运动的干涉
        values = getValues(inputs);
        Infer_val = calmovinferfunc(values{1:17});
        [idx_x, idx_y, idx_z] = ind2sub(size(Infer_val), find(Infer_val == 1));
        scatter3(myAxes0, 50 * (idx_x - 1) - 1000, 50 * (idx_y - 1) - 1000, 2200 - 50 * (idx_z - 1), 50, 'k', 'filled');  % 黑色填充的点
        title(myAxes0, '三维二值点云 (1=黑点)');
        xlabel(myAxes0, 'X'); ylabel(myAxes0, 'Y'); zlabel(myAxes0, 'Z');
        grid on;
        axis equal;
        
    end

    function plot_callback(~, ~)
        if isempty(Length_record)
            msgbox('请先运行程序生成数据！','提示'); return;
        end
        figure('Name', 'Draw Length'); plot(Length_record(:,7:12)); grid on;
        figure('Name', 'Draw Theta1'); plot(Theta_record1(:,7:12)); grid on;
        figure('Name', 'Draw Theta2'); plot(Theta_record2(:,7:12)); grid on;
    end

    function compute_params_callback(~, ~)
        values = cell2mat(getValues(inputs));        
        Gene_ipt = [values(1) * ones(10,1),values(2) * ones(10,1),values(3) * ones(10,1), values(4) * ones(10,1), zeros(10,1)];
        Gene_stat = Gene_ipt;
        [Gene_stat(1,5),~, ~] = calmovingrange(Gene_stat(1,1),Gene_stat(1,2),Gene_stat(1,3),Gene_stat(1,4),values(5),values(6),values(7),values(8),1520,100);
        Gene_stat(:,5) = Gene_stat(1,5);
        for Loop_id = 1:100
            Adapt_rate = 0.5 * (101 - Loop_id)/100;
            Gene_new = zeros(10,5);
            Gene_new(:,1:4) = Gene_stat(:,1:4).*(1 - Adapt_rate + 2 * Adapt_rate * rand(10,4));
            Gene_stat = [Gene_stat;Gene_new];
            for k = 11:20
                [Gene_stat(k,5),~, ~] = calmovingrange(Gene_stat(k,1),Gene_stat(k,2),Gene_stat(k,3),Gene_stat(k,4),values(5),values(6),values(7),values(8),1520,100);
            end
            [~,sort_id] = sort(Gene_stat(:,5),'ascend');
            Gene_stat = Gene_stat(sort_id(1:10),:);
            Randp = randi([1,10],10,4);
            Gene_new = zeros(10,5);
            for k = 1:10
                for j = 1:4
                    Gene_new(k,j) = Gene_stat(Randp(k,j),j);
                end
            end
            Gene_stat = [Gene_stat;Gene_new];
            for k = 11:20
                [Gene_stat(k,5),~, ~] = calmovingrange(Gene_stat(k,1),Gene_stat(k,2),Gene_stat(k,3),Gene_stat(k,4),values(5),values(6),values(7),values(8),1520,100);
            end
            [~,sort_id] = sort(Gene_stat(:,5),'ascend');
            Gene_stat = Gene_stat(sort_id(1:10),:);
            Loop_id
        end
        R_new = Gene_stat(1,1);  % 例：实际应为函数输出
        r_new = Gene_stat(1,2);
        D_new = Gene_stat(1,3);
        d_new = Gene_stat(1,4);
        % 更新文本框
        inputs(1).String = num2str(R_new);
        inputs(2).String = num2str(r_new);
        inputs(3).String = num2str(D_new);
        inputs(4).String = num2str(d_new);
        msgbox('参数计算完成并已更新！','成功');
    end

    function forward_kinematics_callback(~, ~)
        values = cell2mat(getValues(inputs));
        L = cell2mat(getValues(lengthInputs));
        % ============ 正解逻辑区域 =============
        P = zeros(1,6);
        ismove = round(centerMoveCheckbox.Value);
        [P(1,1), P(1,2), P(1,3), P(1,4), P(1,5), P(1,6), Center_pos, Sixgle_pos] = solveSixAxisPlane(values(1),values(2),values(3),values(4),values(5),values(6),values(7),values(8),values(9),values(10),values(11),ismove,L);
        % ========================================
        for i = 1:6
            poseInputs(i).String = num2str(P(1,i));
        end
        msgbox('正解完成！','成功');
    end

    function inverse_kinematics_callback(~, ~)
        values = cell2mat(getValues(inputs));
        P = cell2mat(getValues(poseInputs));
        % ============ 反解逻辑区域 =============
        [Para_opt,Base_pos,Sixgle_pos,Upper_pos] = CalresLength(values(1),values(2),values(3),values(4),values(7)+values(9),values(5)+values(6),values(6),values(6)+values(8),values(10),values(11),P(1),P(2),P(3),P(4),P(5),P(6));
        L = Para_opt(:,1);
        % ========================================
        for i = 1:6
            lengthInputs(i).String = num2str(L(i));
        end
        msgbox('反解完成！','成功');
    end

    function batch_forward_callback(~, ~)
        [file, path] = uigetfile('*.txt', '选择包含6列姿态参数的输入TXT文件');
        if isequal(file,0)
            return;
        end
        inputFile = fullfile(path, file);

        [outfile, outpath] = uiputfile('*.txt', '选择输出TXT文件保存位置');
        if isequal(outfile, 0)
            return;
        end
        outputFile = fullfile(outpath, outfile);

        fid_in = fopen(inputFile, 'r');
        fid_out = fopen(outputFile, 'w');

        if fid_in == -1 || fid_out == -1
            msgbox('无法打开文件','错误','error'); return;
        end

        values = cell2mat(getValues(inputs)); % 获取结构参数
        lineNo = 0;
        %para_guess = zeros(1,6);
        %is_near = 0;
        exact_index = 100000;
        old_inputPose = NaN * ones(1,6);
        same_time = 0;  % 重复次数
        mean_stat = zeros(1,6);
        while ~feof(fid_in)
            line = fgetl(fid_in);
            if ~ischar(line), continue; end
            lineNo = lineNo + 1;
            nums = sscanf(line, '%f');
            if numel(nums) < 50
                fclose(fid_in); fclose(fid_out);
                msgbox(sprintf('第 %d 行少于 50 个数字，终止处理！', lineNo), '错误', 'error');
                return;
            end
            inputPose = nums(5:9:50)';
            para_guess = nums(2:9:47)';
        % 正解计算
            ismove = round(centerMoveCheckbox.Value);
            Times = 0;
            [a, b, c, d, e, f, exact_index] = solveSixAxisPlaneNew(values(1),values(2),values(3),values(4),values(7)+values(9),values(5)+values(6),values(6)+values(8),values(10),values(11),ismove,inputPose,1,para_guess);
            while(exact_index >= 0.01)
                Times = Times + 1;
                if(Times < 10)
                    [a, b, c, d, e, f, exact_index] = solveSixAxisPlaneNew(values(1),values(2),values(3),values(4),vales(5),values(6),values(7),values(8),values(9),values(10),values(11),ismove,inputPose,1,para_guess);
                else
                    [a, b, c, d, e, f, exact_index] = solveSixAxisPlaneNew(values(1),values(2),values(3),values(4),values(5),vales(6),values(7),values(8),values(9),values(10),values(11),ismove,inputPose,0,para_guess);
                end
                fprintf('重做');
            end
            if(norm(old_inputPose - inputPose) <= eps)
                same_time = same_time + 1;
                a = (mean_stat(1) * same_time + a)/(same_time + 1);
                b = (mean_stat(2) * same_time + b)/(same_time + 1);
                c = (mean_stat(3) * same_time + c)/(same_time + 1);
                d = (mean_stat(4) * same_time + d)/(same_time + 1);
                e = (mean_stat(5) * same_time + e)/(same_time + 1);
                f = (mean_stat(6) * same_time + f)/(same_time + 1);
            else
                mean_stat = zeros(1,6);
                same_time = 0;
            end
            fprintf(fid_out, '%.6f %.6f %.6f %.6f %.6f %.6f\n', a, b, c, d, e, f);
            old_inputPose = inputPose;  % 赋值给新的姿态指标
            mean_stat(1) = a;
            mean_stat(2) = b;
            mean_stat(3) = c;
            mean_stat(4) = d;
            mean_stat(5) = e;
            mean_stat(6) = f;
        end
        fclose(fid_in); fclose(fid_out);
        msgbox('批量正解完成！','成功');
    end

    function motion_report_callback(~, ~)
    % 假设 Max_Velocity 和 Max_Accel 是工作区已有变量
    % 可根据需要传入或使用全局变量
        if isempty(Max_Velocity) || isempty(Max_Accel)
            errordlg('请先完成相关计算，生成速度与加速度数据。', '数据缺失');
            return;
        end

    % 格式化输出
        report = sprintf('正方向最大杆长：\n');
        for i = 1:6
            report = [report, sprintf('第%d杆：%.2f，', i, Max_Length(i,1))];
        end
        report(end) = []; report = [report, '\n'];  % 去掉最后一个逗号

        report = [report, '反方向最大杆长：\n'];
        for i = 1:6
            report = [report, sprintf('第%d杆：%.2f，', i, Max_Length(i,2))];
        end
        report(end) = []; report = [report, '\n'];
        
        report = [report, '正方向位置极限：\n'];
        for i = 1:3
            report = [report, sprintf('第%d维度：%.2f，', i, max(Max_pos(:,i)))];
        end
        report(end) = []; report = [report, '\n'];
        
        report = [report, '负方向位置极限：\n'];
        for i = 1:3
            report = [report, sprintf('第%d维度：%.2f，', i, min(Min_pos(:,i)))];
        end
        report(end) = []; report = [report, '\n'];
        
        report = [report, '正方向最大速度：\n'];
        for i = 1:6
            report = [report, sprintf('第%d杆：%.2f，', i, Max_Velocity(i,1))];
        end
        report(end) = []; report = [report, '\n'];  % 去掉最后一个逗号

        report = [report, '反方向最大速度：\n'];
        for i = 1:6
            report = [report, sprintf('第%d杆：%.2f，', i, Max_Velocity(i,2))];
        end
        report(end) = []; report = [report, '\n'];

        report = [report, '正方向最大加速度：\n'];
        for i = 1:6
            report = [report, sprintf('第%d杆：%.2f，', i, Max_Accel(i,1))];
        end
        report(end) = []; report = [report, '\n'];

        report = [report, '正方向最小加速度：\n'];
        for i = 1:6
            report = [report, sprintf('第%d杆：%.2f，', i, Max_Accel(i,2))];
        end
        report(end) = [];

    % 弹窗显示
        msgbox(sprintf(report), '运动报告', 'modal');
    end



    %% ------------------ 工具函数 ------------------
    function values = getValues(controls)
        n = numel(controls);
        values = cell(n,1);
        for k = 1:n
            values{k} = str2double(controls(k).String);
        end
    end

%空间坐标系
% figure('Color','w');
% hold on;
% axis off;


end


