function M = read_space_delimited_matrix(filename)
% READ_SPACE_DELIMITED_MATRIX
% 规则：空格 => 下一列；换行 => 下一行。
% 连续空格会产生空列（以 NaN 记）；各行列数不同时，右侧用 NaN 补齐。

    fid = fopen(filename,'r');
    if fid == -1, error('无法打开文件: %s', filename); end
    cleaner = onCleanup(@() fclose(fid));

    rows = {};
    maxcols = 0;
    line_no = 0;

    while true
        tline = fgetl(fid);
        if ~ischar(tline), break; end
        line_no = line_no + 1;

        tokens = parse_line_tokens_(tline);   % 按单个空格切分，保留空项
        row = nan(1, numel(tokens));
        for k = 1:numel(tokens)
            tok = tokens{k};
            if isempty(tok)
                row(k) = NaN;                 % 空列 -> NaN
            else
                val = str2double(tok);
                if isnan(val) && ~strcmpi(strtrim(tok),'NaN')
                    error('第 %d 行第 %d 列存在非数值内容: "%s".', line_no, k, tok);
                end
                row(k) = val;
            end
        end

        rows{end+1} = row; %#ok<AGROW>
        maxcols = max(maxcols, numel(row));
    end

    M = nan(numel(rows), maxcols);
    for r = 1:numel(rows)
        row = rows{r};
        M(r, 1:numel(row)) = row;
    end
end

% --- 子函数：逐字符分割，严格以单个空格为分隔符，保留连续空格带来的空列 ---
function tokens = parse_line_tokens_(s)
    tokens = cell(0,1);
    curr = '';
    for i = 1:length(s)
        ch = s(i);
        if ch == ' '
            tokens{end+1,1} = curr; %#ok<AGROW>
            curr = '';
        else
            curr = [curr ch];      %#ok<AGROW>  % 行长一般不大，这里直接拼接
        end
    end
    tokens{end+1,1} = curr;        % 收尾
end
