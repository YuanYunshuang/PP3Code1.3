function [ lines, inliers_all ] = findLines( image, num )
[height, width] = size(image);
% line_map = image;
% read the image points, which have intensity>0, into a list
points = zeros(floor(height*width/4),2);
ct = 1;
for i=1:height
    for j=1:width
        if image(i,j)>20
%             line_map(i,j) = 255;
            points(ct,:) = [j,i];
            ct = ct + 1;
        else
%             line_map(i,j) = 0;
        end
    end
end
% imshow(line_map);
% axis([0 height 0 width]);
% xlabel('height[x]');
% ylabel('width[y]');
% axis equal
points(~any(points,2),:)= [];
outliers = points;
inliers_all = zeros(size(points));
n_lines = 0;
start_=1;
lines = zeros(num,3);
n_try = 0;
while(n_lines<num)
    if n_try>20
        break
    end
    n_try = n_try + 1;
    [line,inliers,outliers] = ransac(outliers);
    if length(inliers)<100
        continue;
    end
    % ----density check------
    KDT = KDTreeSearcher(inliers);
%     tmp = norm(line(1:2));
    check = 0;
    
    for j=1:30
%         scatter(inliers(:,1),inliers(:,2),'.','g','LineWidth',0.1);
        rd_p = inliers(randi([1,length(inliers)]),:);
        neighbors = rangesearch(KDT,rd_p,30);
        neighbors = inliers(neighbors{1},:);
%         n_onLine = 0;
%         for k=1:length(neighbors)
%             d = (line(1)*neighbors(k,1)+line(2)*neighbors(k,2)+line(3))/tmp; 
%             if abs(d)<2
%                 n_onLine=n_onLine+1;
%             end
%         end
%         linearity = n_onLine/length(neighbors);
%         if linearity>0.9
         if length(neighbors)>30
            check =  check + 1;
         end
    end
    if check<25
        continue;
    end
    % ---------------------------
    n_lines = n_lines + 1;
    lines(n_lines,:) = line;
    end_ = start_ + length(inliers) - 1;
    inliers_all(start_:end_,:)=inliers;
    start_ = end_ + 1;
    
end
    
% line_map(outliers(:,1),outliers(:,2)) = 0;
inliers_all(~any(inliers_all,2),:)= [];

    
end

            
    
    
    
    






