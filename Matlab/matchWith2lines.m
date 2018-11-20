clc
clear all
close all

lines = readLinesFile('lines_edgy_ransac_depth.txt');
% correlation
map0 = lines(end-1).data;
pc1 = lines(1).data;
n1 = map0(2,1:2);
n2 = pc1(1,1:2);
angle = acos(n1*n2'/(norm(n1)*norm(n2)));
correlation_map = zeros(300,300);
image = imread('../usefulldata/edgy_ransac/depth/cloud0_-10.jpg');
img_ref = imread('../usefulldata/edgy_ransac/depth/map0.jpg');
% img_ref = imread('map0_donNMS.jpg');
% img = imrotate(image,angle*180/pi,'bilinear','crop'); % rotate image clockwise r degree
%---------------------------------------------
point1 = round(intersection(map0(1,:),map0(3,:)));
point2 = round(intersection(pc1(1,:),pc1(3,:)));

t = abs(point2-point1);

%% Plot the result
figure
subplot(2,2,1)
imshow(img_ref);
hold on
plot(point1(1),point1(2),'*');
% plot the lines
for i=1:length(map0)
X = 1:1000;
Y = ceil(-(X.*map0(i,1)+map0(i,3))/map0(i,2));
idx = find(Y>0);
Y = Y(idx);
X = X(idx);
plot(X,Y,'r','LineWidth',1);
end

subplot(2,2,2)
imshow(image);
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
plot(point2(1),point2(2),'*');

%----------------------------------
result = zeros(1000,1000);
% m = min(t(1)+700-1,1000);
% n = min(t(2)+700-1,1000);
% result(1+t(1):m,1+t(2):n) = image(1:m-t(1),1:n-t(2));
result(1:700,1:700) = image;
temp =imtranslate(result,t);
result = rotateAround(temp, point1(2), point1(1), angle*180/pi);
 
I = zeros(1000,1000,3);
I(:,:,1) = img_ref;
I(:,:,2) = img_ref;
I(:,:,3) = img_ref;

I(:,:,1) = result;



subplot(2,2,3)
imshow(result);
subplot(2,2,4)
imshow(I);







    