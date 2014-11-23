%tests to check that induction couplers work

% check that forces are in the correct direction
r = InductionInspector();
n_hat = [0;0;1];
a_body = [0;1;0];
body_rpy = rand(3,100);
for i = 1:size(body_rpy,2)
    q = [0;0;1;body_rpy(:,i)];
    qd = zeros(6,1);
    [force, B_mod, dforce, dB_mod] = r.force{1}.computeSpatialForce(r,q,qd);
    a_world = rpy2rotmat(body_rpy(:,i))*a_body;
    expect_B_mod = zeros(6,1);
    expect_B_mod(1:3) = 1/q(3)^3*(cross(a_world,n_hat));
    
    [tf,errstr] = valuecheck(B_mod,expect_B_mod);
    if ~tf
        disp(errstr)
        disp('index =')
        disp(i)
        break
    end
end




