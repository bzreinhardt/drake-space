classdef InductionInspector < NonlinRigidBodyManipulator
  
  methods
    
    function obj = InductionInspector(urdf,sensor)
      if nargin<1 
          urdf = 'models/two_coupler_inspector.urdf';
          sensor='';
      elseif nargin == 1
          sensor = '';
      end
      options.floating = true;
      %options.terrain = RigidBodyFlatTerrain();
      w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      obj = obj@NonlinRigidBodyManipulator(getFullPathFromRelativePath(urdf),options);
      warning(w);
      
      switch (sensor)
        case 'lidar'
          obj = addFrame(obj,RigidBodyFrame(findLinkInd(obj,'base_link'),[.35;0;0],zeros(3,1),'lidar_frame'));
          lidar = RigidBodyLidar('lidar',findFrameId(obj,'lidar_frame'),-.4,.4,40,10);
          lidar = enableLCMGL(lidar);
          obj = addSensor(obj,lidar);
        case 'kinect'
          obj = addFrame(obj,RigidBodyFrame(findLinkInd(obj,'base_link'),[.35;0;0],zeros(3,1),'kinect_frame'));
          kinect = RigidBodyDepthSensor('kinect',findFrameId(obj,'kinect_frame'),-.4,.4,12,-.5,.5,30,10);
          kinect = enableLCMGL(kinect);
          obj = addSensor(obj,kinect);
      end
      obj = addSensor(obj,FullStateFeedbackSensor);
      % InductionInspectors operate in space
      obj.gravity = [0;0;0];
      obj = compile(obj);
      
    end
   
    function u0 = nominalThrust(obj)
      % each propellor commands -mg/4
      u0 = Point(getInputFrame(obj),getMass(obj)*norm(getGravity(obj))*ones(4,1)/4);
    end
    
    function obj = addObstacles(obj,number_of_obstacles)
      if nargin<2, number_of_obstacles = randi(10); end
      
      for i=1:number_of_obstacles
        xy = randn(2,1);
        while(norm(xy)<1), xy = randn(2,1); end
        height = .5+rand;
        shape = RigidBodyBox([.2+.8*rand(1,2) height],[xy;height/2],[0;0;randn]);
        shape.c = rand(3,1);
        obj = addShapeToBody(obj,'world',shape);
        obj = addContactShapeToBody(obj,'world',shape);
      end
      
      obj = compile(obj);
    end
    
    function obj = addTrees(obj,number_of_obstacles)
      % Adds a random forest of trees
      if nargin<2, number_of_obstacles = 5*(randi(5)+2); end
      for i=1:number_of_obstacles
        % Populates an area of the forest
        xy = [20,0;0,12]*(rand(2,1) - [0.5;0]);
        % Creates a clear path through the middle of the forest
        while(norm(xy)<1 || (xy(1,1)<=1.5 && xy(1,1)>=-1.5)), xy = randn(2,1); end
        height = 1+rand;
        width_param = rand(1,2);
        yaw = randn;
        obj = obj.addTree([width_param height],xy,yaw);
      end
      obj = compile(obj);
    end
    
    function obj = addTree(obj, lwh, xy, yaw)
      % Adds a single tree with specified length width height, xy
      % location, and yaw orientation.
      height = lwh(1,3);
      width_param = lwh(1,1:2);
      treeTrunk = RigidBodyBox([.2+.8*width_param height],...
          [xy;height/2],[0;0;yaw]);
      treeTrunk.c = [83,53,10]/255;  % brown
      obj = addShapeToBody(obj,'world',treeTrunk);
      obj = addContactShapeToBody(obj,'world',treeTrunk);
      treeLeaves = RigidBodyBox(1.5*[.2+.8*width_param height/4],...
          [xy;height + height/8],[0;0;yaw]);
      treeLeaves.c = [0,0.7,0];  % green
      obj = addShapeToBody(obj,'world',treeLeaves);
      obj = addContactShapeToBody(obj,'world',treeLeaves);
      obj = compile(obj);
    end  
    
    function obj = addTargetSurface(obj,lwh,xyz,rpy)
        if nargin <4
            rpy = zeros(3,1);
            if nargin <3
                xyz = zeros(3,1);
            end
        end
        %Adds a conductive surface 
        plate = RigidBodyBox(lwh,xyz,rpy);
        plate.c = [150,150,150]/255;
        obj = addShapeToBody(obj,'world',plate);
   %     obj = addContactShapeToBody(obj,'world',plate);
        obj = compile(obj);
    end
    
    function obj = addCurvedSurface(obj,rad,len,varargin)
         % obj = addCurvedSurface(radius,len) adds a
      % RigidBodyCylinder to the world link. It defaults to aligning with
      % the X axis (so the circle lies in the Y-Z plane.)
      %
      % obj =addCurvedSurface(radius,len,T) constructs a
      % RigidBodyCylinder object with the geometry-to-body transform T.
      % 
      % obj = addCurvedSurface(radius,len,xyz,rpy) constructs a
      % RigidBodyCylinder object with the geometry-to-body transform
      % specified by the position, xyz, and Euler angles, rpy.
      %
      % @param radius - radius of the cylinder
      % @param len - length of the cylinder
      % @param T - 4x4 homogenous transform from geometry-frame to
      %   body-frame
      % @param xyz - 3-element vector specifying the position of the
      %   geometry in the body-frame
      % @param rpy - 3-element vector of Euler angles specifying the
      %   orientation of the geometry in the world-frame
        if nargin == 3
            varargin = {[0;0;-7],[0;pi/2;0]};
        end
        plate = RigidBodyCylinder(rad,len,varargin{:});
        %plate = RigidBodyCapsule(rad,len,varargin{:});
        plate.c = [150,150,150]/255;
        obj = addShapeToBody(obj,'world',plate);
        obj = compile(obj);
    end
        
    
    function traj_opt = addPlanVisualizer(obj,traj_opt)
      % spew out an lcmgl visualization of the trajectory.  intended to be
      % used as a callback (fake objective) in the direct trajectory
      % optimization classes

      if ~checkDependency('lcmgl')
        warning('lcmgl dependency is missing.  skipping visualization'); 
        return;
      end
      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'InductionInspectorPlan');
      
      typecheck(traj_opt,'DirectTrajectoryOptimization');

      traj_opt = traj_opt.addDisplayFunction(@(x)visualizePlan(x,lcmgl),traj_opt.x_inds(1:3,:));
      
      function visualizePlan(x,lcmgl)
        lcmgl.glColor3f(1, 0, 0);
        lcmgl.glPointSize(3);
        lcmgl.points(x(1,:),x(2,:),x(3,:));
        lcmgl.glColor3f(.5, .5, 1);
        lcmgl.plot3(x(1,:),x(2,:),x(3,:));
        lcmgl.switchBuffers;
      end
    end
  end
  
  
  methods (Static)
    
    function runOpenLoop
      r = InductionInspector();
     r.setGravity([0;0;0]) %we're in space
     r = r.addTargetSurface([5,5,0.01],[0;0;0],[0;0;0]);
      sys = TimeSteppingRigidBodyManipulator(r,.01);
      
     v = sys.constructVisualizer();

      x0 = [0;0;0.5;zeros(13,1)];
      u0 = Point(getInputFrame(r),[0.08; -0.06]);
      
      sys = cascade(ConstantTrajectory(u0),sys);

%      sys = cascade(sys,v);
%      simulate(sys,[0 2],double(x0)+.1*randn(12,1));
      
      options.capture_lcm_channels = 'LCMGL';
      [ytraj,xtraj,lcmlog] = simulate(sys,[0 5],double(x0),options);
      
   %   [ytraj2,xtraj2] = simulateODE(sys,[0,2],double(x0),options)
      lcmlog
      %v.playback(xtraj,struct('lcmlog',lcmlog));
      figure(1); clf; fnplt(ytraj,3);
      v.playback(xtraj);
    end
    
    
  end
end