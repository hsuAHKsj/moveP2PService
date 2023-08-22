function [start_end_points, start_theta, end_theta] = readStartEndPoints(filename)
    % 输入：文件名 filename
    % 输出：起始点的矩阵 start_end_points，起始点的角度 start_theta 和终止点的角度 end_theta
    
    % 读取 JSON 文件数据
    json_str = fileread(filename);
    
    % 解析 JSON 数据为结构体
    data = jsondecode(json_str);
    
    % 获取起始点数据
    start_point = data.StartPoint';
    end_point = data.EndPoint';
    
    % 获取角度信息
    start_theta = data.StartTheta;
    end_theta = data.EndTheta;
    
    % 构建起始点矩阵
    start_end_points = [start_point; end_point];
end
