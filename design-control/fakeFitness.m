function fitness = fakeFitness(inspector)
%FAKEFITNESS tries to minimize the distance between each arm and the y axis
%to test the design algorithm
typecheck(inspector, 'Inspector2d');
fitness = 0;

for i = 1:size(inspector.d,2)
    dist = abs(dot(inspector.d(:,i),[0;1;0]))/norm(inspector.d(:,i));
    fitness = fitness + dist;
end
%normalize for number of arms
fitness = fitness/size(inspector.d,2);
end