% 提取指定列并保存到新文件 RecLen.txt
% 假设 input.txt 用空格或制表符分隔
clear; clc;

% 输入输出文件名
inputFile  = 'input.txt';
outputFile = 'RecLen.txt';

% 打开文件
fid_in  = fopen(inputFile, 'r');
fid_out = fopen(outputFile, 'w');

% 检查文件是否打开成功
if fid_in == -1
    error('无法打开输入文件 %s', inputFile);
end
if fid_out == -1
    error('无法创建输出文件 %s', outputFile);
end

% 要提取的列号
cols = [5, 14, 23, 32, 41, 50, 2, 11, 20, 29, 38, 47];

% 按行读取
line = fgetl(fid_in);
while ischar(line)
    % 按空格/制表符分割
    tokens = strsplit(strtrim(line));
    
    % 确保这一行有足够的列
    if length(tokens) >= max(cols)
        % 提取所需列
        selected = tokens(cols);
        % 写入输出文件，用空格分隔
        fprintf(fid_out, '%s ', selected{:});
        fprintf(fid_out, '\n');
    end
    
    % 读下一行
    line = fgetl(fid_in);
end

% 关闭文件
fclose(fid_in);
fclose(fid_out);

disp('数据提取完成，结果已保存到 RecLen.txt');
