function select_every_50th_line(input_file, output_file)
    % 打开输入文件
    fid = fopen(input_file, 'r');
    if fid == -1
        error('无法打开输入文件 %s', input_file);
    end

    % 读取所有行
    lines = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    lines = lines{1};

    % 选取每第50行（即第1、51、101...行）
    selected_lines = lines(1:50:end);

    % 打开输出文件写入
    fid_out = fopen(output_file, 'w');
    for i = 1:length(selected_lines)
        fprintf(fid_out, '%s\n', selected_lines{i});
    end
    fclose(fid_out);
end
