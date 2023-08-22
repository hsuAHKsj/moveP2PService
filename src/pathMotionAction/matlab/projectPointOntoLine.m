function projection = projectPointOntoLine(point, start_point, end_point)
% 计算点在线段上的投影
% 输入：需要投影的点，以及线段的起始点和结束点
% 输出：投影点的坐标

% 计算向量
vector = end_point - start_point;
vector_norm = vector / norm(vector);

% 计算投影
projection = start_point + ((point - start_point) * vector_norm') * vector_norm;

end
