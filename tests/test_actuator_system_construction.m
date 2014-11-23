%test actuator system
num_xc = 0;
num_xd = 0;
direct_feedthrough_flag = true;
time_invariant_flag = true;
fake_in_frame = CoordinateFrame('fake',2,[],{'u1','u2'});
%Create two rigidBodyManipulators
%Put one inside the ActuatorSystem
file = 'models/two_coupler_inspector.urdf';

plant = InductionInspector(file);
target = InductionInspector(file);
num_actuators = length(plant1.force);

plant_output = getOutputFrame(plant1);
input_frame = MultiCoordinateFrame({fake_in_frame,plant_output.frame{:}},[]);
output_frame = getInputFrame(plant1);
act = ActuatorSystem(num_xc,num_xd,input_frame,output_frame,direct_feedthrough_flag,time_invariant_flag,num_actuators);
act = setTarget(act,target);

state = plant1.output(0,[0; 0; 1; zeros(15,1)],zeros(2,1));
input = [1;0];
output = act.output(0,0,[input;state]);

% input_select(1).system = 1;
% input_select(1).input = 1;
% connection1(1).from_output(1) = 1;
% connection1(1).to_input = 1;
 connection2(1).to_input = 2;
 connection2(1).from_output = 1;
 connection2(2).from_output = 2;
 connection2(2).to_input = 3;
combined_system = mimoFeedback(act, plant1, [],connection2,[],[]);
c = lqr(combined_system);
