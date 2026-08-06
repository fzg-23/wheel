clc; clear;
close all;

% 读取 CSV 文件
% 文件名1 = 'logdata_squat_good.csv'; % CSV 文件名
%文件名2 = 'logdata_squat_good_soft.csv'; % CSV 文件名
% 文件名2 = 'logdata_forward_backward.csv'; % CSV 文件名
% 文件名2 = 'logdata_forward_backward_soft.csv'; % CSV 文件名
% 文件名1 = 'logdata_onstand_soft.csv'; % CSV 文件名
% 文件名2 = 'logdata_handstand_soft_2.csv'; % CSV 文件名

filename1 = 'logdata_yawing.csv'; % CSV 文件名
filename2 = 'logdata_yawing_soft.csv'; % CSV 文件名
% 文件名2 = 'logdata_yawing_soft_2.csv'; % CSV 文件名

plot_fft_data(filename1, filename2);
