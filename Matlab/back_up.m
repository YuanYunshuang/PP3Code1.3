clc
clear all
close all

img_reg = imread('map0_deri.jpg');
%img_ref = imresize(img_ref,0.4);
% img_reg = imread('cloud0_-10.jpg');
%img_reg = imresize(img_reg, 0.4);
% [height_ref, width_ref] = size(img_ref);
[height, width] = size(img_reg);

% read the image points, which have intensity>0, into a list
points = zeros(height*width/4,2);
figure 
xlabel('height[x]');
ylabel('width[y]');
axis equal
hold on

ct = 1;
for i=1:height
    for j=1:width
        if img_reg(i,j)>20
%             img_reg(i,j) = 255;
            points(ct,:) = [i,j];
            ct = ct + 1;
        end
    end
end

points(~any(points,2),:)= [];
scatter(points(:,1),points(:,2),'.','k','LineWidth',0.05);
n_points = length(points);
KDT = KDTreeSearcher(points);
% RANSAC
Num_Itr = 1000;
count = zeros(Num_Itr,4);
for i=1:Num_Itr
    p1 = points(randi([1,n_points]),:);
    [neighbors,dists] = rangesearch(KDT,p1,40);
    neighbors = neighbors{1};
    dists = dists{1};
    t=0;
    while(t<1000)
        id = randi([1,length(neighbors)]);
        p2 = points(neighbors(id),:);
        if dists(id)>20
            break;
        end
        t=t+1;
    end
    if t>=1000
        continue;
    end
    S_x1 = [  0 -1  p1(2);
             1  0 -p1(1);
            -p1(2) p1(1) 0];
    l = S_x1*[p2(1);p2(2);1];
    tmp = norm(l(1:2));
    inlier_ids = zeros(n_points,1);
    ct = 0;
    for j=1:n_points
        p = points(j,:);
        d = (l(1)*p(1)+l(2)*p(2)+l(3))/tmp;
        if abs(d)<5
            ct = ct+1;
            inlier_ids(j) = 1;
        end
    end
    count(i,1:3) = l;
    count(i,4) = ct;

%     plot(p1(1),p1(2),'*','LineWidth',2,'MarkerSize',10)
%     plot(p2(1),p2(2),'*','LineWidth',2,'MarkerSize',10)
%     inliers = points;
%     inliers(logical(~inlier_ids),:) = [];
%     scatter(inliers(:,1),inliers(:,2),'.','g','LineWidth',0.1);
    
end
% find the best result
[value, idx]=max(count(:,4));
line = count(idx,1:3);
% plot the line
X = 1:height;
Y = ceil(-(X.*line(1)+line(3))/line(2));
idx = find(Y>0);
Y = Y(idx);
X = X(idx);
p = plot(X,Y,'r','LineWidth',2);
            
    
    
    
    