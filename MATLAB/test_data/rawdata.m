clc; clear;
close all;

% CSV 파일 읽기
% filename1 = 'logdata_squat_good.csv'; % CSV 파일 이름
% filename2 = 'logdata_squat_good_soft.csv'; % CSV 파일 이름
% filename2 = 'logdata_forward_backward.csv'; % CSV 파일 이름
% filename2 = 'logdata_forward_backward_soft.csv'; % CSV 파일 이름
% filename1 = 'logdata_onstand_soft.csv'; % CSV 파일 이름
% filename2 = 'logdata_handstand_soft_2.csv'; % CSV 파일 이름

filename1 = 'logdata_yawing.csv'; % CSV 파일 이름
filename2 = 'logdata_yawing_soft.csv'; % CSV 파일 이름
% filename2 = 'logdata_yawing_soft_2.csv'; % CSV 파일 이름

plot_fft_data(filename1, filename2);
