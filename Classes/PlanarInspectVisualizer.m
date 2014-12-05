classdef PlanarInspectVisualizer < Visualizer
% Implements the draw function for the Planar Quadrotor model

  properties
   sphere_radius;
   sphere_center;
   d;
   a;
   L;
  end

  methods
    function obj =  PlanarInspectVisualizer(plant)
      typecheck(plant,'PlanarInspector');
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.sphere_radius = plant.sphere_radius;
      obj.sphere_center = plant.sphere_center;
   obj.d =plant.d;
   obj.a =plant.a;
   obj.L = 0.1;
  
   
    end
    
    function draw(obj,t,x)
      % Draw the quadrotor.  
      persistent hFig base arms mags;

      if (isempty(hFig))
        hFig = sfigure(25);
        set(hFig,'DoubleBuffer', 'on');
        
        base = [0.5*obj.L*[1 -1 -1 1]; 0.5*obj.L*[1 1 -1 -1]];
        arms = zeros(2,size(obj.d,1)*4);
        for i = 1:size(obj.d,1)
            theta = atan2(obj.d(i,2),obj.d(i,1));
            arm = [norm(obj.d(i,1:2))/2*[1 1 -1 -1]; .01*[1 0 0 1]];
            r = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            arm_w = r*arm+obj.d(i,1:2)'/2*ones(1,4);
            arms(:,4*i-3:4*i) = arm_w;
        end
        mags = obj.d(:,1:2)';
      end
            
      sfigure(hFig); cla; hold on; view(0,90);
      
      r = [cos(x(3)), -sin(x(3)); sin(x(3)), cos(x(3))];
      
      
      
      for i = 1:size(arms,2)/4
           p = r*[arms(1,4*i-3:4*i);arms(2,4*i-3:4*i)];
            patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 0]);
            
           p = r*base;
      patch(x(1)+p(1,:), x(2)+p(2,:),1+0*p(1,:),'b','FaceColor',[.6 .6 .6])
            
      end
      rad = 0.01;
      for i = 1:size(mags,2)
           p = r*[mags(:,i)];
            draw2DCircle(p+[x(1);x(2)],rad,'b','FaceColor',[200 200 200]/255);
            
      end
      
      draw2DCircle(obj.sphere_center, obj.sphere_radius, 'b','FaceColor',[150 150 150]/255)
      
      title(['t = ', num2str(t(1),'%.2f') ' sec']);
      %set(gca,'XTick',[],'YTick',[])
      
      axis image; axis([-0.7 0.7 -0.1 1.0]);
      drawnow;
    end    
  end
  
end