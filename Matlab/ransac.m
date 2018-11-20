function [ line, inliers, outliers ] = ransac( points )

% scatter(points(:,1),points(:,2),'.','k','LineWidth',0.05);
n_points = length(points);
KDT = KDTreeSearcher(points);
% RANSAC
Num_Itr = 1000;
scores = zeros(Num_Itr,4);
for i=1:Num_Itr
    p1 = points(randi([1,n_points]),:);
    [neighbors,dists] = rangesearch(KDT,p1,20);
    neighbors = neighbors{1};
    dists = dists{1};
    t=0;
    while(t<1000)
        id = randi([1,length(neighbors)]);
        p2 = points(neighbors(id),:);
        if dists(id)>10
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
        if abs(d)<4
            ct = ct+1;
            inlier_ids(j) = 1;
        end
    end
    scores(i,1:3) = l;
    scores(i,4) = ct;
    
end % main loop

% find the best result
[value, idx]=max(scores(:,4));
line = scores(idx,1:3);
tmp = norm(line(1:2));
inlier_ids = zeros(n_points,1);
for j=1:n_points
    p = points(j,:);
    d = (line(1)*p(1)+line(2)*p(2)+line(3))/tmp;
    if abs(d)<5
        inlier_ids(j) = 1;
    end
end


outliers = points;
outliers(logical(inlier_ids),:) = [];
inliers = points;
inliers(logical(~inlier_ids),:) = [];

end

