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