function [main_channels,lines] = readMainChannels(filename)
    % 输入：文件名 filename
    % 输出：主通道信息的结构体数组 main_channels
    
    % 读取 JSON 文件数据
    json_str = fileread(filename);
    
    % 解析 JSON 数据为结构体数组
    data = jsondecode(json_str);
    
    % 初始化主通道结构体数组
    num_channels = numel(data);
    main_channels = struct('ID', {}, 'StartPoint', {}, 'EndPoint', {}, 'Width', {});
    
    % 解析每个主通道的信息
    for i = 1:num_channels
        channel_data = data(i);
        
        % 读取通道的ID、起点、终点和宽度
        id = channel_data.ID;
        start_point = channel_data.StartPoint';
        end_point = channel_data.EndPoint';
        width = channel_data.Width;
        
        % 将通道信息添加到主通道结构体数组中
        main_channels(i).ID = id;
        main_channels(i).StartPoint = start_point;
        main_channels(i).EndPoint = end_point;
        main_channels(i).Width = width;
    end


    % 初始化主通道起点和终点的矩阵
    main_channels_lines = zeros(num_channels, 4);
    
    % 解析每个主通道的信息
    for i = 1:num_channels
        channel_data = data(i);
        
        % 读取通道的起点和终点
        start_point = channel_data.StartPoint';
        end_point = channel_data.EndPoint';
        
        % 将起点和终点组合为一个矩阵，并存储到 cell 数组中
        lines{i} = [start_point; end_point];
    end
    
end
