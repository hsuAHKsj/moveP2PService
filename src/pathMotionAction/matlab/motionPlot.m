close all;
clear all;
data = load('../data/1.txt');
% inputData = dlmread('../data/input.txt');% 读取input.txt文件

figure;
axis equal;
hold on;
grid on;
xlabel('x');
ylabel('y');

% 提取x、y、theta列
outx = data(:, 1);
outy = data(:, 2);
outtheta = data(:,3);
outcolor = data(:, 4);

% 定义小车模型
carLength = 1; % 小车长度
carWidth = 0.5; % 小车宽度
car = [
    -carLength/2, -carWidth/2; % 左下角
    carLength/2, -carWidth/2; % 右下角
    carLength/2, carWidth/2; % 右上角
    -carLength/2, carWidth/2; % 左上角
];

% 定义车头箭头长度和角度
arrowLength = 0.5;
arrowAngle = pi/6; % 30度

% 创建一个 VideoWriter 对象
videoFile = VideoWriter('motion_animation.mp4', 'MPEG-4');
open(videoFile);


% 创建动画
for i = 1:length(outx)
    % 清空图形
    clf;
    
    % 绘制路径
    plot(outx, outy, 'r-');
    grid on;
    hold on;
    title("小车实时运动仿真");
    
    % 计算小车模型的旋转矩阵
    R = [cos(outtheta(i)), -sin(outtheta(i));
         sin(outtheta(i)), cos(outtheta(i))];
    
    % 计算小车模型在当前位置的坐标
    carPos = [outx(i); outy(i)];
    carVertices = (R * car' + carPos)';
    
    % 绘制小车模型
    patch(carVertices(:, 1), carVertices(:, 2), 'g');
    
    % 计算车头箭头的坐标
    arrowPos = [outx(i); outy(i)];
    arrowEnd = arrowPos + arrowLength * [cos(outtheta(i)); sin(outtheta(i))];
    arrowLeft = arrowPos + arrowLength * [cos(outtheta(i) + arrowAngle); sin(outtheta(i) + arrowAngle)];
    arrowRight = arrowPos + arrowLength * [cos(outtheta(i) - arrowAngle); sin(outtheta(i) - arrowAngle)];
    
    % 绘制车头箭头
    plot([arrowPos(1), arrowEnd(1)], [arrowPos(2), arrowEnd(2)], 'r-');
    plot([arrowEnd(1), arrowLeft(1)], [arrowEnd(2), arrowLeft(2)], 'r-');
    plot([arrowEnd(1), arrowRight(1)], [arrowEnd(2), arrowRight(2)], 'r-');
    
    % 设置坐标轴范围
    xlim([min(outx) - 1, max(outx) + 1]);
    ylim([min(outy) - 1, max(outy) + 1]);
    
    % 暂停一小段时间，控制动画速度
    pause(0.005);
    % 将当前图形帧写入视频文件
    frame = getframe(gcf);
    writeVideo(videoFile, frame);
end

% 关闭视频文件
close(videoFile);
