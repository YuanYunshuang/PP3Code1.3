function [ point ] = intersection( l1,l2 )
det = l1(1)*l2(2) - l2(1)*l1(2);

if abs(det)<0.01
    point=[0 0]; %parallel
else
    point(1)=(l2(3)*l1(2) - l1(3)*l2(2))/det;
    point(2)=(l2(1)*l1(3) - l1(1)*l2(3))/det;
end

end

