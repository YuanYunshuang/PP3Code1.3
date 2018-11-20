clc
clear all
close all


% read images
image_path = '../usefulldata/images_edgy/depth/';
save_path = '../usefulldata/edgy_ransac/depth/';
images = dir(image_path);
images = images(4:end);
fid = fopen(strcat(save_path,'lines.txt'),'w');
for i=1:length(images)
    name = images(i).name;
    img = imread(strcat(image_path,name));
    [height, width] = size(img);
    [ lines, inliers_all ] = findLines(img,3);
    
    line_map = zeros(height, width);
    for j=1:length(inliers_all)
        line_map(inliers_all(j,2),inliers_all(j,1)) = 1;
    end
    imwrite(line_map,strcat(save_path,name),'jpg');
    fprintf(fid,name);
    fprintf(fid,'\n');
    for k=1:length(lines)
        fprintf(fid,'%8.4f %8.4f %8.4f\n',lines(k,:));
    end
    %--------
    figure(1)
    axis([0 700 0 700]);
    scatter(inliers_all(:,1),inliers_all(:,2),'.','g','LineWidth',0.1);
    hold on
    % plot the lines
    for i=1:length(lines)
    X = 1:height;
    Y = ceil(-(X.*lines(i,1)+lines(i,3))/lines(i,2));
    idx = find(Y>0);
    Y = Y(idx);
    idx = find(Y<height);
    Y = Y(idx);
    X = X(idx);
    plot(X,Y,'r','LineWidth',2);
    end
    %---------
    figure(2)
    imshow(line_map);
    hold on
    % plot the lines
    for i=1:length(lines)
    X = 1:height;
    Y = ceil(-(X.*lines(i,1)+lines(i,3))/lines(i,2));
    idx = find(Y>0);
    Y = Y(idx);
    X = X(idx);
    plot(X,Y,'r','LineWidth',2);
    end
end
fclose(fid);





