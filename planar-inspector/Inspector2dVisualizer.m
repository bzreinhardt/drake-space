classdef Inspector2dVisualizer < Visualizer
% Implements the draw function for the Planar Quadrotor model

  properties
   sphere_radius;
   sphere_center;
   d;
   a;
   L;
   num_couplers;
   new;

   fig_num;
  end
  
  methods
      function obj =  Inspector2dVisualizer(plant,fig_num)
          typecheck(plant,'Inspector2d');
        
          obj = obj@Visualizer(plant.getOutputFrame);
            if nargin < 2
              obj.fig_num = 25;
          else
              obj.fig_num = fig_num;
          end

          obj.sphere_radius = plant.sphere_radius;
          obj.sphere_center = plant.sphere_center;
          obj.d =plant.d;
          obj.a =plant.a;
          obj.L = 0.1;
          obj.num_couplers = plant.getNumInputs;
          obj.new = true;
          
          
      end
      
      function draw(obj,t,x)
          % Draw the Inspector
      persistent hFig base arms mags;

      if (obj.new)

        hFig = sfigure(obj.fig_num);

        set(hFig,'DoubleBuffer', 'on');
        
        base = [0.5*obj.L*[1 -1 -1 1]; 0.5*obj.L*[1 1 -1 -1]];
        arms = zeros(2,size(obj.d,2)*4);
        for i = 1:obj.num_couplers
            %angle between arm frame and body frame
            theta = atan2(obj.d(2,i),obj.d(1,i));
            %corners of arm rectangle 
            arm = [norm(obj.d(1:2,i))/2*[1 1 -1 -1]; .01*[1 0 0 1]];
            %rotation matrix
            r = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            %arm in body frame
            arm_w = r*arm+obj.d(1:2,i)/2*ones(1,4);
            arms(:,4*i-3:4*i) = arm_w;
        end
        mags = obj.d(1:2,:);
        obj.new = false; 
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
      
      axis image; axis([-0.3 0.3 -0.1 0.5]);
      drawnow;
    end    
  end
  
end