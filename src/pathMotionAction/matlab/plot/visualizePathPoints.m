function visualizePathPoints(P)
    % 输入：
    % P: 最短路径示教点坐标数组
    % start_point: 起始点坐标
    % end_point: 终止点坐标
    
%     P = [start_point; P ; end_point];
    start_point = P(1,:);
    end_point = P(end,:);
    % 绘制起始点和终止点
    plot(start_point(1), start_point(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(end_point(1), end_point(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    
    % 添加起始点和终止点的备注
    text(start_point(1), start_point(2), '起始点', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    text(end_point(1), end_point(2), '终止点', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    
    % 绘制最短路径示教点
    plot(P(:, 1), P(:, 2), 'yo-', 'LineWidth', 2);
    
    % 添加最短路径示教点的备注
    for i = 2:size(P, 1)-1
        text(P(i, 1), P(i, 2), sprintf('示教点 %d', i), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    end
   
end
