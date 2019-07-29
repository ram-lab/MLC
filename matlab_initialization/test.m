addpath('/home/jjiao/Documents/matlab_ws/Making-elegant-Matlab-figures/Boxplot');


% Number of intended boxes in the figure
num_boxes = 8;          

% Generating random data
data = cell(1,num_boxes);   
for k = 1:num_boxes
    data{k} = randi(10) + randn(1,1000);
end

% Using the "figure_boxplot.m" function to plot the boxplot figure using the data

figure_boxplot(data)