% 读取 CSV 文件
function xyThetaList = readRefVector(filename)
    % 读取 CSV 文件
    data = readtable(filename', 'Delimiter', ',');

    % 解析数据
    xyThetaList.x = data.x;
    xyThetaList.y = data.y;
    xyThetaList.theta = data.theta;
    xyThetaList.prop = data.prop;
end
