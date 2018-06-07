clear all ;
clc;
close all;

base_file = 'AtlasBaseCOMHeightCompare.mat';
toe_file = 'AtlasToeCOMHeightCompare.mat';

toe_height = load(toe_file,'q_z');
base_height = load(base_file,'q_z');

base_l_start_bend = load(base_file,'q_z');

plot(base_height.q_z, 'b');
hold on;
plot(toe_height.q_z, 'r');
hold off;
title('COM Height');

fprintf('Mean of base is %d \n',mean(base_height.q_z)*100);
fprintf('Mean of toe is %d \n',mean(toe_height.q_z)*100);

fprintf('Variance of base is %d \n',var(base_height.q_z)*100);
fprintf('Variance of toe is %d \n',var(toe_height.q_z)*100);