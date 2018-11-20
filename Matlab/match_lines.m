clc
clear all

lines = readLinesFile('lines_edgy_ransac_depth.txt');
% correlation
map0 = lines(end-1).data;
pc1 = lines(1).data;
n1 = map0(2,1:2);
n2 = pc1(1,1:2);
angle = acos(n1*n2'/(norm(n1)*norm(n2)));
correlation_map = zeros(300,300);
image = imread('../usefulldata/images/depth/cloud0_10.jpg');
img_ref = imread('map0_donNMS.jpg');
img = imrotate(image,angle*180/pi,'bilinear','crop'); % rotate image clockwise r degree

h=300;
figure
subplot(2,2,1)
imshow(img_ref);
subplot(2,2,2)
imshow(image);
subplot(2,2,3)
imshow(img)
% for x=1:2:h
%     tic
%     for y=1:2:h 
%         correlation_map(x,y) = sum(sum(img.*img_ref(x:(x+700-1),y:(y+700-1))));
%     end
%     toc
% end
% 
% [X idx_x] = max(correlation_map(:,:));
% [Y ty] = max(X);
% tx = idx_x(ty);

result = zeros(1000,1000);
result(tx:tx+700-1,ty:ty+700-1) = img;
 
I = zeros(1000,1000,3);
I(:,:,1) = img_ref;
I(:,:,2) = img_ref;
I(:,:,3) = img_ref;

I(:,:,1) = result;



subplot(2,2,3)
imshow(result);
subplot(2,2,4)
imshow(I);







    