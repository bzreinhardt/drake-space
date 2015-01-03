function intersect = checkTwoBB(bb1,bb2)
case1 = and(bb1(:,1) <= bb2(:,1),bb1(:,2)>=bb2(:,1));
case2 = and(bb1(:,1) >= bb2(:,1),bb1(:,1)<=bb2(:,2));
intersect = all(or(case1,case2));


end