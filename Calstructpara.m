%% 推导目前的最佳结构参数
%% 推荐参数数值：R = 2157.10/2;  r = 1209.70/2;  D = 300/2;   d = 250/2
Gene_ipt = [2147.1/2 * ones(10,1),1209.7/2 * ones(10,1),150 * ones(10,1), 125 * ones(10,1), zeros(10,1)];
Gene_stat = Gene_ipt;
[Gene_stat(1,5),~, ~] = calmovingrange(Gene_stat(1,1),Gene_stat(1,2),Gene_stat(1,3),Gene_stat(1,4),1520,100);
Gene_stat(:,5) = Gene_stat(1,5);
Adapt_rate = 0.5;
for Loop_id = 1:100
    Adapt_rate = 0.1 * (101 - Loop_id)/100;
    Gene_new = zeros(10,5);
    Gene_new(:,1:4) = Gene_stat(:,1:4).*(1 - Adapt_rate + 2 * Adapt_rate * rand(10,4));
    Gene_stat = [Gene_stat;Gene_new];
    for k = 11:20
        [Gene_stat(k,5),~, ~] = calmovingrange(Gene_stat(k,1),Gene_stat(k,2),Gene_stat(k,3),Gene_stat(k,4),1520,100);
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
        [Gene_stat(k,5),~, ~] = calmovingrange(Gene_stat(k,1),Gene_stat(k,2),Gene_stat(k,3),Gene_stat(k,4),1520,100);
    end
    [~,sort_id] = sort(Gene_stat(:,5),'ascend');
    Gene_stat = Gene_stat(sort_id(1:10),:);
    Loop_id
end
