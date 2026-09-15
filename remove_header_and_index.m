function remove_header_and_index(input_file, output_file)
    % 打开输入文件
    fid = fopen(input_file, 'r');
    if fid == -1
        error('无法打开输入文件 %s', input_file);
    end

    % 读取所有行
    lines = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    lines = lines{1};

    % 去掉第一行表头
    lines(1) = [];

    % 处理每一行：去掉开头的序号，只保留数值
    data = [];
    for i = 1:length(lines)
        tokens = textscan(lines{i}, '%f');
        values = tokens{1};
        % 去掉第一列（序号）
        values(1) = [];
        data = [data; values'];
    end

    % 写入输出文件
    dlmwrite(output_file, data, 'delimiter', '\t', 'precision', '%.4f');
end
