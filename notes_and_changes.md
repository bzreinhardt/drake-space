Added induction_coupler to RigidBodyManipulator - need more elegant way of added custom force elements:q
Need to modify constructing a frame for the rigid body coupler (RigidBodyElement line 1883) - need a less hacky way to do this

RigidBodyThrusts, RigidBodyPropellers and RigidBodyCouplers are special because they have their own frames associated with them

Interesting difference between actuators and force elements

Note difference between v and qd in robotics

Need to manually compile an object after setting a property that changes the 'dirty' property, like gravity

Setgravity function does not seem to work

What's the difference between rigidBodyThrust and rigidBodyForce?

DTTrajectory.fnplt(plotdims) - plots the trajectory of a DTTrajectory object (which is what's spit out by simualte)

Methods and who they're called by

RigidBodyCoupler.computeSpatialForce called by manipulatorDynamics@RigidBodyManipulator

computeSpatialForce is the money function for defining how a RigidBodyForceElement works

T in RigidBodyFrames and RigidBodyObjects is the transform matrix 
 obj.T = [rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),xyz; 0,0,0,1]; where rotz will rotate about the z axis by rpy(3), etc.. to multiplying [x;y;z;1] will transform INTO the body frame

 valuecheck() is an amazing function

 B and B_mod are in the world reference frame 

 rpy2rotmat gives the matrix that will convert a vector represented in body coordinates with that given rpy to a representation in world coordinate

 lcmgl is the java 3D drawing function I believe

 Things to invest

 FindClosestPoints only works on the level of distinct RigidBody objects in a model
 Example

 Trajectory Optimization
 ======================
 geval.m for Drake gradient format
 dynamics needs 2 outputs xcdot and dxcdot
 * note that in polynomialsystem (and I believe rigidbodysystem) xcdot and dxcdot are generated symbolically
 *Trajectories can be stored in PPTrajectory
 *To create a PPTrajectory you need a pp form spline
 *Drake has two(+) functions for creating ppform splines from a series of data. It uses pchipDeriv(t,x,xdot) to create position trajectory. It uses foh(t,u) to create input trajectories.
 *pchipDeriv(t,y,ydot_minus,ydot_plus,dim) @optional ydot_plus,dim finds a cubic spline that fits the input data
  it implements piecewise cubic Hermite polynomials with the derivatives
  % specified.  (e.g., as in Hargraves86).
  *ypp = foh(t0,y0) creates a ppspline of first order derivatives between the specified points

  USEFUL FUNCTIONS I DIDN"T KNOW ABOUT
  ====================================
  cumsum(A) - turns vector into cumulative running sum of vector elements


