function boundary = create_boundary(start_point, end_point, r)
% 创建线段通道边界
% 输入：线段的起始点 start_point，线段的结束点 end_point，通道的半径 r
% 输出：线段通道的边界，包括上下边界以及上下半圆边界

% 解构起始点和结束点的坐标
x1 = start_point(1);
y1 = start_point(2);
x2 = end_point(1);
y2 = end_point(2);

% 计算法向量
normal_vector = [y1 - y2, x2 - x1];

% 计算法向量的长度
length = norm(normal_vector);

% 计算法向量的标准化向量
normalized_vector = normal_vector / length;

% 计算移动向量
movement_vector = r * normalized_vector;

% 计算半圆的圆心坐标
center1 = [x1, y1];
center2 = [x2, y2];

% 计算半圆的起始角度和结束角度
start_angle = atan2(normalized_vector(2), normalized_vector(1));
end_angle = start_angle + pi;

% 设置通道端点处的半圆边界参数
theta = linspace(start_angle, end_angle, 100); % 在起始角度和结束角度之间生成一系列角度
x_circle = r * cos(theta); % 半圆上点的 x 坐标
y_circle = r * sin(theta); % 半圆上点的 y 坐标

% 设置通道的上下边界绘图参数
upper_x1 = x1 + movement_vector(1);
upper_y1 = y1 + movement_vector(2);
upper_x2 = x2 + movement_vector(1);
upper_y2 = y2 + movement_vector(2);

down_x1 = x1 - movement_vector(1);
down_y1 = y1 - movement_vector(2);
down_x2 = x2 - movement_vector(1);
down_y2 = y2 - movement_vector(2);

% 返回所有边界信息
boundary.upper_boundary = [upper_x1, upper_y1; upper_x2, upper_y2];
boundary.lower_boundary = [down_x1, down_y1; down_x2, down_y2];
boundary.upper_semi_circle = center1 + [x_circle; y_circle]';
boundary.lower_semi_circle = center2 - [x_circle; y_circle]';

end
