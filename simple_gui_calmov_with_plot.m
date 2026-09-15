function simple_gui_calmov_with_plot
    % 创建主界面窗口
    fig = figure('Name', 'CalmovFunc参数输入', 'Position', [300 200 800 450]);

    % 参数名（内部用）
    paramNames = {'R', 'r', 'D', 'd', ...
                  'Base_H', 'Center_H', 'Upper_Radis', 'UpperJ_Radis', ...
                  'Rx_Amp', 'Ry_Amp', 'Rz_Amp', ...
                  'X_Amp', 'Y_Amp', 'Z_Amp'};

    % 中文提示
    chineseLabels = {'静平台半径', '动平台半径', '静平台铰点距离', '动平台铰点距离', ...
                     '静平台上边高', '回转中心高度', '动平台回转半径', '动铰链回转半径', ...
                     '翻滚角幅值', '纵倾角幅值', '偏航角幅值', ...
                     'X方向幅值', 'Y方向幅值', 'Z方向幅值'};

    % 默认值
    defaultValues = [2157.10/2, 1209.70/2, 300/2, 250/2, ...
                     193, 2600, 899.2, 1080.2, ...
                     0, 0, 0, 0, 0, 0];

    % 创建输入框
    inputs = gobjects(length(paramNames),1);

    % 布局参数
    x_start = 30;
    y_start = 370;
    x_spacing = 180;
    y_spacing = 70;

    for i = 1:4
        xpos = x_start + (i-1)*x_spacing;
        ypos = y_start;
        createInput(i, xpos, ypos);
    end
    for i = 5:8
        xpos = x_start + (i-5)*x_spacing;
        ypos = y_start - y_spacing;
        createInput(i, xpos, ypos);
    end
    for i = 9:11
        xpos = x_start + (i-9)*x_spacing;
        ypos = y_start - 2*y_spacing;
        createInput(i, xpos, ypos);
    end
    for i = 12:14
        xpos = x_start + (i-12)*x_spacing;
        ypos = y_start - 3*y_spacing;
        createInput(i, xpos, ypos);
    end

    % 创建按钮
    uicontrol('Style', 'pushbutton', 'String', '运行', ...
              'Position', [220 20 100 40], 'FontSize', 12, 'Callback', @run_callback);
    uicontrol('Style', 'pushbutton', 'String', '计算参数', ...
              'Position', [340 20 100 40], 'FontSize', 12, 'Callback', @compute_params_callback);
    uicontrol('Style', 'pushbutton', 'String', '绘制曲线', ...
              'Position', [460 20 100 40], 'FontSize', 12, 'Callback', @plot_callback);

    % 初始化数据变量
    Length_record = [];
    Theta_record1 = [];
    Theta_record2 = [];

    function createInput(index, xpos, ypos)
        uicontrol('Style', 'text', 'Position', [xpos ypos+25 140 20], ...
                  'String', chineseLabels{index}, 'HorizontalAlignment', 'left', 'FontSize', 10);
        inputs(index) = uicontrol('Style', 'edit', 'Position', [xpos ypos 100 25], ...
                                  'FontSize', 10, 'String', num2str(defaultValues(index)));
    end

    function run_callback(~, ~)
        values = zeros(length(paramNames),1);
        for j = 1:length(paramNames)
            values(j) = str2double(inputs(j).String);
        end
        [Length_record, Theta_record1, Theta_record2] = calmovfunc(values(1), values(2), values(3), values(4), ...
                                                                    values(5), values(6), values(7), values(8), ...
                                                                    values(9), values(10), values(11), ...
                                                                    values(12), values(13), values(14));
        disp('运行完成！');
        msgbox('运行完成，数据已保存到工作区！','成功');
        assignin('base', 'Length_record', Length_record);
        assignin('base', 'Theta_record1', Theta_record1);
        assignin('base', 'Theta_record2', Theta_record2);
    end

    function compute_params_callback(~, ~)
        values = zeros(length(paramNames),1);
        for j = 1:length(paramNames)
            values(j) = str2double(inputs(j).String);
        end
        
        
        
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
        % ===========================

        % 更新文本框
        inputs(1).String = num2str(R_new);
        inputs(2).String = num2str(r_new);
        inputs(3).String = num2str(D_new);
        inputs(4).String = num2str(d_new);

        msgbox('参数计算完成并已更新！','成功');
    end

    function plot_callback(~, ~)
        if isempty(Length_record)
            msgbox('请先点击运行按钮，生成数据！', '提示');
            return;
        end
        figure('Name', 'draw length');
        plot(Length_record(:,7:12));
        title('Draw Length');
        xlabel('Time Step');
        ylabel('Length Value');
        legend({'L1','L2','L3','L4','L5','L6'});
        grid on;

        figure('Name', 'draw theta1');
        plot(Theta_record1(:,7:12));
        title('Draw Theta1');
        xlabel('Time Step');
        ylabel('Theta1 Value');
        legend({'T1','T2','T3','T4','T5','T6'});
        grid on;

        figure('Name', 'draw theta2');
        plot(Theta_record2(:,7:12));
        title('Draw Theta2');
        xlabel('Time Step');
        ylabel('Theta2 Value');
        legend({'T1','T2','T3','T4','T5','T6'});
        grid on;

        disp('绘图完成！');
    end
end
