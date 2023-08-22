function [task_end_point, task_projection] = task_channel_projection(main_channel, task_end, task_radius, id_name)
    % 输入：主通道的端点 main_channel，任务通道的终点 task_end，任务通道的半径 task_radius
    % 输出：任务通道的终点和任务通道在主通道上的投影点
    
    % 提取主通道的端点
    start_point = main_channel(1, :);
    end_point = main_channel(2, :);
    
    % 计算任务通道终点在主通道上的投影点
    task_projection = projectPointOntoLine(task_end, start_point, end_point);
    disp(start_point)
    
    % 在图中标记任务通道的终点和投影点
    hold on;
    % 创建任务通道的边界
    task_boundary = create_boundary(task_end, task_projection, task_radius);
    % 绘制任务通道的边界
    plot_boundary(task_boundary, start_point, end_point);
% %     
    plot(task_end(1), task_end(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2); % 任务通道的终点
    plot(task_projection(1), task_projection(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % 投影点
    
    % 在图中绘制从任务通道的终点到投影点的线段
    line([task_end(1), task_projection(1)], [task_end(2), task_projection(2)], 'Color', 'g', 'LineStyle', '--', 'LineWidth', 2);
    
    % 添加投影线段的备注名称
    name = ['Task ', num2str(id_name)];
%     text(mean([task_end(1), task_projection(1)]), mean([task_end(2), task_projection(2)]), name, 'Color', 'red', 'FontSize', 12, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    % 返回任务通道的终点和投影点
    task_end_point = task_end;
end
