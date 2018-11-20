clc
clear all
close all

img_ref = imread('../images/map0.jpg');
img_ref = imresize(img_ref,0.4);
img_reg = imread('../images/0/cloud0_-10.jpg');
img_reg = imresize(img_reg, 0.4);
[height_ref, width_ref] = size(img_ref);
[height, width] = size(img_reg);

correlation_map = zeros(300,300,2);

for x=1:3:height_ref-height
    tic
    for y=1:3:width_ref-width
        corr = zeros(360,1);
        for r=1:360
            img = imrotate(img_reg,-r,'bilinear','crop'); % rotate image clockwise r degree
            corr(r) = sum(sum(img.*img_ref(x:(x+height-1),y:(y+width-1))))/(height*width);
            correlation_map(x,y,:) = max(corr);
        end
    end
    toc
end

maximum = max(max(correlation_map(:,:,1)));
[t_x,t_y]=find(correlation_map(:,:,2)==maximum);
rot = correlation_map(t_x,t_y,1);
        