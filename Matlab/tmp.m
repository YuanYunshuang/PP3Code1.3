clc
clear all
close all


% read images
img = imread('map0_deri.jpg');

[height, width] = size(img);
[ lines, inliers_all ] = findLines(img,6);

line_map = zeros(height, width);
for j=1:length(inliers_all)
    line_map(inliers_all(j,1),inliers_all(j,2)) = 1;
end
scatter(inliers_all(:,1),inliers_all(:,2),'.','g','LineWidth',0.1);
axis([0 700 0 700]);
hold on
% plot the lines
for k=1:length(lines)
X = 1:height;
Y = ceil(-(X.*lines(k,1)+lines(k,3))/lines(k,2));
idx = find(Y>0);
Y = Y(idx);
X = X(idx);
plot(X,Y,'r','LineWidth',2);
end







