
close all;
clear all;
clc;
addpath fileOperation
addpath plot

figure
grid on;
axis equal;
main_channel_file = '../config/mainChannel.json';
task_channels_file = '../config/taskChannels.json';
[main_channels,lines] = readMainChannels(main_channel_file);
task_channels = readTaskChannels(main_channel_file, task_channels_file);
start_end_file = '../config/startEndPoints.json';
[start_end_points, start_theta, end_theta]= readStartEndPoints(start_end_file); % 读取起始点 终止点 从json数据包

P  = readmatrix('../data/pathInfo.csv');
visualizePathPoints(P);

xyThetaList = readRefVector( '../data/refVector.csv');
arrowLength = 0.1;
plotRefVector(xyThetaList, arrowLength)