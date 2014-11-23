%test halbach graphing
%Example of using magnet field graphing functions
%% Halbach array
c = InductionCouplerModel();
c.P = 2;
figure(2);
clf;
X0 = [0;0.05];
plotHalbachField(c,X0);
xlim([-0.06,0.06]);ylim([-0.01,0.11])
set(gca,'xtick',[],'ytick',[])
%save2pdf('/home/ben/Documents/papers/two_d_inspection_paper/figures/halbach_array.pdf');



%% PM array
rot = rpy2rotmat([0,0,pi/4]);

centers = rot(1:2,1:2)*[[0;c.r_o],[0;-c.r_o]]+X0*ones(1,2);
directions = rot(1:2,1:2)*[[0;1],[0;-1]];

figure(1);clf;



            

rectangle('Position',[X0(1)-c.r_o,X0(2)-c.r_o,2*c.r_o,2*c.r_o],...
                'Curvature',[1 1],'FaceColor',[205 201 201]/255);
            hold on;
c.plotArray(directions,centers);
%draw the plate
            rectangle('Position',[-0.1,-0.01,0.2,0.01],'FaceColor',[205 201 201]/255);
            start_theta = linspace(0,2*pi,100);
            xlim([-0.06,0.06]);ylim([-0.01,0.11])
            set(gca,'xtick',[],'ytick',[])
hold off;
%save2pdf('/home/ben/Documents/papers/two_d_inspection_paper/figures/pm_array.pdf');

            
