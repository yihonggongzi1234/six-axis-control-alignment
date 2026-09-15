function min_distance = calculate_min_segment_distance(segments)
    % 示例输入：六个线段的起点和终点坐标,共计六行（格式：[xb, yb, zb, xe, ye, ze]）
    % 你可以修改这些值来测试不同的线段组合    
    % 初始化最小距离为无穷大
    min_distance = inf;
    % 循环遍历所有线段对（避免重复计算）
    for i = 1:5
        for j = i+1:6
            % 提取线段i和线段j的起点和终点
            p1 = segments(i, 1:3); q1 = segments(i, 4:6);
            p2 = segments(j, 1:3); q2 = segments(j, 4:6);
            % 计算线段间的最短距离
            min_distance = min(min_distance, segment_distance(p1, q1, p2, q2));
        end
    end
end

function d = segment_distance(p1, q1, p2, q2)
    % 计算两条线段之间的最短距离（不包括延长线）

    u = q1 - p1;
    v = q2 - p2;
    w = p1 - p2;

    a = dot(u, u);
    b = dot(u, v);
    c = dot(v, v);
    d_ = dot(u, w);
    e = dot(v, w);
    denom = a * c - b * b;

    if denom ~= 0
        s = (b * e - c * d_) / denom;
        s = max(0, min(1, s));
    else
        s = 0;
    end

    t = (b * s + e) / c;
    if t < 0
        t = 0;
        s = max(0, min(1, -d_ / a));
    elseif t > 1
        t = 1;
        s = max(0, min(1, (b - d_) / a));
    end

    pt1 = p1 + s * u;
    pt2 = p2 + t * v;
    d = norm(pt1 - pt2);
end
