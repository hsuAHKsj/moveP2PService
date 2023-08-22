function task_channels = readTaskChannels(main_channel_file, task_channels_file)
    % 输入：主通道文件名 main_channel_file，任务通道文件名 task_channels_file
    % 输出：任务通道信息的结构体数组 task_channels
    
    % 读取主通道信息
    main_channels = readMainChannels(main_channel_file);
    
    % 读取任务通道信息
    json_str = fileread(task_channels_file);
    task_channels_data = jsondecode(json_str);
    
    % 初始化任务通道结构体数组
    num_channels = numel(task_channels_data);
    task_channels = struct('ID', {}, 'MainChannelID', {}, 'StartPoint', {}, 'EndPoint', {}, 'Radius', {});
    
    % 解析每个任务通道的信息
    for i = 1:num_channels
        channel_data = task_channels_data(i);
        
        % 读取任务通道的ID、终点、属主通道ID和半径
        id = channel_data.ID;
        end_point = channel_data.EndPoint';
        main_channel_id = channel_data.MainChannelID;
        radius = channel_data.Radius;
        
        % 根据属主通道ID获取对应的主通道信息
        main_channel = getMainChannelEndpoints(main_channel_id, main_channels); 

        % 计算任务通道的投影点（即任务通道上的起点）
        [end_point,start_point] = task_channel_projection(main_channel, end_point, radius,id);

        % 将通道信息添加到任务通道结构体数组中
        task_channels(i).ID = id;
        task_channels(i).MainChannelID = main_channel_id;
        task_channels(i).StartPoint = start_point;
        task_channels(i).EndPoint = end_point;
        task_channels(i).Radius = radius;
    end
end

function line_segment = getMainChannelEndpoints(main_channel_id, main_channels)
    % 输入：主通道ID main_channel_id，主通道信息数组 main_channels
    % 输出：主通道ID对应的起始点和终止点组成的线段 line_segment
    
    % 初始化起始点和终止点
    start_point = [];
    end_point = [];
    
    % 遍历主通道信息数组
    for i = 1:numel(main_channels)
        if main_channels(i).ID == main_channel_id
            % 获取起始点和终止点
            start_point = main_channels(i).StartPoint;
            end_point = main_channels(i).EndPoint;
            break;
        end
    end
    
    % 构建线段
    line_segment = [start_point; end_point];
end


