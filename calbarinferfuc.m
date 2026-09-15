function [Infer_val] = calbarinferfuc(Infer_val, beg_P, end_P, radis)
%   此处显示详细说明
    x_min = min(beg_P(1), end_P(1));
    x_max = max(beg_P(1), end_P(1));
    y_min = min(beg_P(2), end_P(2));
    y_max = max(beg_P(2), end_P(2));
    z_min = min(beg_P(3), end_P(3));
    z_max = max(beg_P(3), end_P(3));
    min_vec = [x_min - radis, y_min - radis, z_min - radis];
    max_vec = [x_max + radis, y_max + radis, z_max + radis];
    for x = 1:41
        x_p = (x - 1) * 50 - 1000;
        for y = 1:41
            y_p = (y - 1) * 50 - 1000;
            for z = 1:41
                z_p = 200 + (z-1) * 50;
                if(Infer_val(x,y,z) == 0)
                    Pt = [x_p,y_p,z_p];
                    scale_f = (Pt - min_vec)/(max_vec - min_vec);
                    if((max(scale_f) <= 1) && (min(scale_f) >= 0))
                        dist = pointToSegmentDistanceBuiltin(Pt, beg_P, end_P);
                        if(dist <= radis)
                            Infer_val(x,y,z) = 1;
                        end
                    end
                end
            end
        end
    end
end


function dist = pointToSegmentDistanceBuiltin(Pt, beg_P, end_P)
% 使用 MATLAB 内置函数的高效版本

    % 将线段参数化：f(t) = beg_P + t*(end_P - beg_P), t ∈ [0,1]
    
    AB = end_P - beg_P;
    AP = Pt - beg_P;
    
    % 计算投影参数
    t = dot(AP, AB) / dot(AB, AB);
    
    % 找到线段上最近的点
    if((t < 0) || (t > 1))
        dist = 10000;
    else
        closest_point = beg_P + t * AB;
    
    % 返回距离
        dist = norm(Pt - closest_point);
    end
end
