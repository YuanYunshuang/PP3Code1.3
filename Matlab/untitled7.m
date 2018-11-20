load('0_-10.Mat');

image_o = imread('../usefulldata/images/depth/cloud0_-10.jpg');
img_ref_o = imread('../usefulldata/images/depth/map0.jpg');

%% Plot the result
figure
subplot(2,2,1)
imshow(img_ref);
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
plot(point1(1),point1(2),'*','LineWidth',3);
%-----------------
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
plot(point2(1),point2(2),'*','LineWidth',3);

%----------------------------------
result = zeros(1000,1000);
% m = min(t(1)+700-1,1000);
% n = min(t(2)+700-1,1000);
% result(1+t(1):m,1+t(2):n) = image(1:m-t(1),1:n-t(2));
result(1:700,1:700) = image_o;
temp =imtranslate(result,t);
result = rotateAround(temp, point1(2), point1(1), angle*180/pi);
 
I = zeros(1000,1000,3);
I(:,:,1) = img_ref_o;
I(:,:,2) = img_ref_o;
I(:,:,3) = img_ref_o;

I(:,:,1) = result;

subplot(2,2,3)
imshow(result);
subplot(2,2,4)
imshowpair(img_ref_o,result);