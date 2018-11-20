clc
clear all
close all

lines = readLinesFile('lines_edgy_ransac_depth.txt');
% image = imread('../usefulldata/images/depth/cloud0_10.jpg');
image = imread('../usefulldata/depth_pca/cloud0_-40.jpg');
image = addBuffer( image, 0);
% imshow(image);
% img_ref = imread('../usefulldata/depth/depth/map0.jpg');
img_ref = imread('map0_donNMS.jpg');
img_ref = addBuffer( img_ref, 0);
% img_ref = imread('map0_donNMS.jpg');
% img = imrotate(image,angle*180/pi,'bilinear','crop'); % rotate image clockwise r degree
%---------------------------------------------
%% calculate trasformation for a pair of point cloud and map images
map0 = lines(end-1).data;
pc1 = lines(4).data;
% loop on different line matchings and line intersections
angles = zeros(6,0);
ct=1;
for i=1:3
    for j=i:3
        n1 = map0(i,1:2);
        n2 = pc1(j,1:2);
        angles(ct) = acos(n1*n2'/(norm(n1)*norm(n2)));
        ct = ct + 1;
    end
end

points1 = zeros(2,2);
points2 = zeros(2,2);
ct1 = 1;
ct2 = 1;
for m=1:3
    for n=m+1:3
        point1 = round(intersection(map0(m,:),map0(n,:)));
        point2 = round(intersection(pc1(m,:),pc1(n,:)));
        if point1(1)>0 && point1(2)>0 && point1(1)<1000 && point1(2)<1000
            points1(ct1,:) = point1;
            ct1 = ct1 + 1;
        end
        if point2(1)>0 && point2(2)>0 && point2(1)<700 && point2(2)<700
            points2(ct2,:) = point2;
            ct2 = ct2 + 1;
        end
    end
end

match_results = zeros(4,8);
ct=1;
for i=1:2 % point in map
    for j=1:2 % point in pc
        t = abs(points2(j,:)-points1(i,:));
        result = zeros(1000,1000);
        result(1:700,1:700) = image;
        temp =imtranslate(result,t);
        angle = 0;
        corr_old = 0;
        for k=1:6 % loop over intersection angles
            %criterion for the result:correlation
            %----------------------------------
            result = rotateAround(temp, points1(i,2), points1(i,1), angles(k)*180/pi);
            n_non0 = nnz(result);
            corr = img_ref.*result;
            corr = sum(sum(corr))/n_non0;
            if corr>corr_old
                angle = angles(k);
                corr_old = corr;
            end
        end
        match_results(ct,:) = [t points1(i,:) points2(j,:) angle corr_old];
        ct = ct + 1;
    end
end

[max_corr max_idx] = max(match_results(:,8));
trafo = match_results(max_idx,:);

%% Plot the result
figure
title('result for matching cloud0_-10.jpg');
subplot(2,2,1)
imshow(img_ref);
title('lines of map0 after ransac');
hold on
% plot the lines
for i=1:length(map0)
X = 1:1000;
Y = ceil(-(X.*map0(i,1)+map0(i,3))/map0(i,2));
idx = find(Y>0);
Y = Y(idx);
X = X(idx);
plot(X,Y,'r','LineWidth',1);
end
plot(trafo(3), trafo(4),'*','LineWidth',4,'MarkerSize',10);

subplot(2,2,2)
imshow(image);
title('lines of point cloud after ransac');
hold on
% plot the lines
for i=1:length(map0)
X = 1:700;
Y = ceil(-(X.*pc1(i,1)+pc1(i,3))/pc1(i,2));
idx = find(Y>0);
Y = Y(idx);
X = X(idx);
plot(X,Y,'r','LineWidth',1);
end

hold on
plot(trafo(5), trafo(6),'*','LineWidth',4,'MarkerSize',10);

result = zeros(1000,1000);
result(1:700,1:700) = image;
temp =imtranslate(result,trafo(1:2));
result = rotateAround(temp, trafo(4), trafo(3), trafo(7)*180/pi);
 
I = zeros(1000,1000,3);
I(:,:,1) = img_ref;
I(:,:,2) = img_ref;
I(:,:,3) = img_ref;

I(:,:,1) = result;



subplot(2,2,3)
imshow(result);
title('point cloud image transformed');
subplot(2,2,4)
imshow(I);
title('transformed point cloud over map0');







    