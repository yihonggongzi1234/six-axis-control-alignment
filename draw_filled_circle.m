function draw_filled_circle(center, normal, radius)
    % 生成单位圆
    theta = linspace(0, 2*pi, 100);
    x = radius * cos(theta);
    y = radius * sin(theta);
    z = zeros(size(x));

    % 计算旋转矩阵，使 [0,0,1] 旋转到给定法线方向
    normal = normal / norm(normal); % 归一化法线
    up = [-0.001, 0.001, 1]; % 默认法线
    v = cross(up, normal); % 旋转轴
    s = norm(v);
    c = dot(up, normal);
    Vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0]; % 反对称矩阵
    R = eye(3) + Vx + Vx^2 * ((1 - c) / s^2); % 罗德里格旋转矩阵

    % 旋转点
    rotated_points = R * [x; y; z];

    % 平移到圆心
    X = rotated_points(1, :) + center(1);
    Y = rotated_points(2, :) + center(2);
    Z = rotated_points(3, :) + center(3);

    % 绘制填充圆
    fill3(X, Y, Z, 'c', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
end


