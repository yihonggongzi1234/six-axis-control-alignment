function draw_six_axis_realtime(baseRadius, platformRadius, basePoints, jointPoints, platformPoints)
%DRAW_SIX_AXIS_REALTIME Draw Figure 2 in the mechanism coordinate system.
% Right-handed frame: X points forward/out of the screen, Y points up,
% and Z points left. X-Z remains the platform plane.

    figure(2);
    clf;
    ax = axes('Parent', gcf);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');

    baseCenter = mean(basePoints, 1);
    jointCenter = mean(jointPoints, 1);
    platformCenter = mean(platformPoints, 1);
    baseNormal = [0, 1, 0];
    platformNormal = plane_normal(jointPoints, baseNormal);

    % Base and moving platform.  Both are rendered from their real X/Y/Z
    % coordinates; no Z-to-height remapping or sign inversion is applied.
    draw_disk(ax, baseCenter, baseNormal, baseRadius, [0.10 0.60 0.90], 0.18);
    draw_disk(ax, jointCenter, platformNormal, platformRadius, [0.95 0.35 0.20], 0.25);
    draw_disk(ax, platformCenter, platformNormal, platformRadius, [0.95 0.35 0.20], 0.25);
    draw_platform_wall(ax, jointCenter, platformCenter, platformNormal, platformRadius);

    % Six hydraulic rods and their numbered moving joints.
    for index = 1:6
        plot3(ax, [basePoints(index,1), jointPoints(index,1)], ...
            [basePoints(index,2), jointPoints(index,2)], ...
            [basePoints(index,3), jointPoints(index,3)], ...
            'k-', 'LineWidth', 2.5);
        plot3(ax, basePoints(index,1), basePoints(index,2), basePoints(index,3), ...
            'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 5);
        plot3(ax, jointPoints(index,1), jointPoints(index,2), jointPoints(index,3), ...
            'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
        text(ax, jointPoints(index,1), jointPoints(index,2), jointPoints(index,3), ...
            sprintf('  %d', index), 'Color', 'r', 'FontWeight', 'bold');
    end
    % Fixed global coordinate frame and motion mapping:
    % +X = X translation and RX rotation, +Y = height and RY rotation,
    % +Z = Z translation and RZ rotation.
    allPoints = [basePoints; jointPoints; platformPoints];
    pointMin = min(allPoints, [], 1);
    pointMax = max(allPoints, [], 1);
    plotRange = max(pointMax - pointMin);
    axisLength = max(100, 0.20 * plotRange);
    % Put the coordinate triad at the lower plot corner so it remains
    % readable and does not obscure the real-time rods or moving platform.
    origin = pointMin - 0.08 * plotRange;
    axisEnd = origin + axisLength;
    labelGap = 0.08 * axisLength;
    quiver3(ax, origin(1), origin(2), origin(3), axisLength, 0, 0, 0, ...
        'Color', [0.85 0.10 0.10], 'LineWidth', 2.0, 'MaxHeadSize', 0.4);
    quiver3(ax, origin(1), origin(2), origin(3), 0, axisLength, 0, 0, ...
        'Color', [0.10 0.55 0.15], 'LineWidth', 2.0, 'MaxHeadSize', 0.4);
    quiver3(ax, origin(1), origin(2), origin(3), 0, 0, axisLength, 0, ...
        'Color', [0.10 0.25 0.90], 'LineWidth', 2.0, 'MaxHeadSize', 0.4);
    text(ax, axisEnd(1)+labelGap, origin(2), origin(3), 'X', ...
        'Color', [0.85 0.10 0.10], 'FontSize', 14, ...
        'FontWeight', 'bold', 'Clipping', 'off');
    text(ax, origin(1), axisEnd(2)+labelGap, origin(3), 'Y', ...
        'Color', [0.10 0.55 0.15], 'FontSize', 14, ...
        'FontWeight', 'bold', 'Clipping', 'off');
    text(ax, origin(1), origin(2), axisEnd(3)+labelGap, 'Z', ...
        'Color', [0.10 0.25 0.90], 'FontSize', 14, ...
        'FontWeight', 'bold', 'Clipping', 'off');
    text(ax, 0.02, 0.97, 'X: \DeltaX / R_X', ...
        'Units', 'normalized', 'Color', [0.85 0.10 0.10], ...
        'FontWeight', 'bold', 'BackgroundColor', 'w', 'Margin', 2);
    text(ax, 0.02, 0.91, 'Y: \DeltaY / R_Y   [height]', ...
        'Units', 'normalized', 'Color', [0.10 0.55 0.15], ...
        'FontWeight', 'bold', 'BackgroundColor', 'w', 'Margin', 2);
    text(ax, 0.02, 0.85, 'Z: \DeltaZ / R_Z', ...
        'Units', 'normalized', 'Color', [0.10 0.25 0.90], ...
        'FontWeight', 'bold', 'BackgroundColor', 'w', 'Margin', 2);

    xlabel(ax, 'X');
    ylabel(ax, 'Y (height)');
    zlabel(ax, 'Z');
    title(ax, 'Real-time six-axis mechanism');
    view(ax, 125, 20);
    set(ax, 'CameraUpVector', [0, 1, 0]);

    margin = 0.16 * plotRange;
    xlim(ax, [pointMin(1)-margin, pointMax(1)+margin]);
    ylim(ax, [pointMin(2)-margin, pointMax(2)+margin]);
    zlim(ax, [pointMin(3)-margin, pointMax(3)+margin]);
    drawnow limitrate;
    hold(ax, 'off');
end

function normal = plane_normal(points, referenceNormal)
    normal = cross(points(2,:) - points(1,:), points(3,:) - points(1,:));
    normal = normal / norm(normal);
    if dot(normal, referenceNormal) < 0
        normal = -normal;
    end
end

function draw_disk(ax, center, normal, radius, color, alpha)
    [basisU, basisV] = disk_basis(normal);
    angle = linspace(0, 2*pi, 100);
    ring = center + radius * (cos(angle(:))*basisU + sin(angle(:))*basisV);
    fill3(ax, ring(:,1), ring(:,2), ring(:,3), color, ...
        'FaceAlpha', alpha, 'EdgeColor', color, 'LineWidth', 1.5);
end

function draw_platform_wall(ax, lowerCenter, upperCenter, normal, radius)
    [basisU, basisV] = disk_basis(normal);
    angle = linspace(0, 2*pi, 100);
    lowerRing = lowerCenter + radius * (cos(angle(:))*basisU + sin(angle(:))*basisV);
    upperRing = upperCenter + radius * (cos(angle(:))*basisU + sin(angle(:))*basisV);
    surface(ax, [lowerRing(:,1), upperRing(:,1)], ...
        [lowerRing(:,2), upperRing(:,2)], [lowerRing(:,3), upperRing(:,3)], ...
        'FaceColor', [0.95 0.35 0.20], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
end

function [basisU, basisV] = disk_basis(normal)
    seed = [1, 0, 0];
    if abs(dot(seed, normal)) > 0.9
        seed = [0, 0, 1];
    end
    basisU = cross(normal, seed);
    basisU = basisU / norm(basisU);
    basisV = cross(normal, basisU);
end
