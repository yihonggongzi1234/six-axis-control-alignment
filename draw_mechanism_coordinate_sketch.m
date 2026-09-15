function draw_mechanism_coordinate_sketch(ax)
%DRAW_MECHANISM_COORDINATE_SKETCH Draw the six-actuator layout.
% Right-handed frame: X points forward/out of the screen, Y points up,
% and Z points left. X-Z remains the platform plane.
% Positive rotations: Roll=RX (alpha), Yaw=RY (beta), Pitch=RZ (gamma).

    cla(ax);
    hold(ax, 'on');
    axis(ax, 'off');
    grid(ax, 'off');

    radius = 5;
    axisLength = 10;
    angle = linspace(0, 2*pi, 180);

    % X-Z is the plan view and Y is the vertical direction.
    quiver3(ax, 0, 0, 0, axisLength, 0, 0, 0, 'k-', ...
        'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(ax, 0, 0, 0, 0, axisLength, 0, 0, 'k-', ...
        'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(ax, 0, 0, 0, 0, 0, axisLength, 0, 'k-', ...
        'LineWidth', 2, 'MaxHeadSize', 0.5);

    plot3(ax, radius*cos(angle), zeros(size(angle)), radius*sin(angle), ...
        'k-', 'LineWidth', 1.5);

    % Actuator numbering matches the supplied top view.
    jointAngleDeg = [15, -15, -105, -135, 135, 105];
    jointX = radius*cosd(jointAngleDeg);
    jointY = zeros(1, 6);
    jointZ = radius*sind(jointAngleDeg);
    plot3(ax, jointX, jointY, jointZ, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
    for index = 1:6
        text(ax, jointX(index)+0.55, jointY(index), jointZ(index), ...
            num2str(index), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r');
    end
    text(ax, axisLength*1.08, 0, 0, 'X / R_X', ...
        'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'middle');
    text(ax, 0, axisLength*1.08, 0, 'Y / R_Y', ...
        'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom');
    text(ax, 0, 0, axisLength*1.08, 'Z / R_Z', ...
        'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'middle');
    text(ax, 0, 0, 0, 'O', 'FontSize', 14, 'FontWeight', 'bold');

    % A small reference line makes the X-Z plan-view orientation explicit.
    plot3(ax, [0 sqrt(21)], [0 0], [0 1.2], 'b--', 'LineWidth', 1.5);
    plot3(ax, [0 sqrt(21)], [0 0], [0 -1.2], 'b--', 'LineWidth', 1.5);

    daspect(ax, [1 1 1]);
    view(ax, 125, 20);
    % Apply this after VIEW, because VIEW resets MATLAB's camera-up vector.
    set(ax, 'CameraUpVector', [0 1 0]);
    hold(ax, 'off');
end
