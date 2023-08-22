function plot_boundary(boundary, start_point, end_point)
% 绘制边界并填充内部，并画出线段
% 输入：边界结构体，包含上下边界以及上下半圆边界的信息
%       线段的起始点和结束点

hold on;

% 绘制线段
plot([start_point(1), end_point(1)], [start_point(2), end_point(2)], 'b', 'LineWidth', 2);


% 绘制上下边界
plot(boundary.upper_boundary(:,1), boundary.upper_boundary(:,2), 'r', 'LineWidth', 2);
plot(boundary.lower_boundary(:,1), boundary.lower_boundary(:,2), 'r', 'LineWidth', 2);

% 绘制上下半圆边界
plot(boundary.upper_semi_circle(:,1), boundary.upper_semi_circle(:,2), 'r', 'LineWidth', 2);
plot(boundary.lower_semi_circle(:,1), boundary.lower_semi_circle(:,2), 'r', 'LineWidth', 2);
end
