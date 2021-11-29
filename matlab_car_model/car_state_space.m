%car matlab state space
%global y position
%y_dot = Vx*yaw+V*y
%x = [y_dot, yaw, yaw_dot]
%u = front steering angle

%longitudinal velocity at the center of gravity of vehicle
Vx = 15;
%mass of vehicle in kg
m = 1575;
%Yaw moment of inertia of vehicle
Iz = 2875;
%longitudinal distance from the center of gravity to front tires
lf = 1.2;
%longitudinal distance from the center of gravity to rear tires
lr = 1.6;
%cornering stiffnes of tire
Cf = 19000;
Cr = 33000;

A = [-(2*Cf+2*Cr)/m/Vx, 0, -Vx-(2*Cf*lf-2*Cr*lr)/m/Vx, 0;
     0, 0, 1, 0;
     -(2*Cf*lf-2*Cr*lr)/Iz/Vx, 0, -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx, 0;
     1, Vx, 0, 0];
B = [2*Cf/m 0 2*Cf*lf/Iz 0]';
C = [0 0 0 1; 0 1 0 0];
vehicle = ss(A,B,C,0);

%horizon N=3
x = [0; 0; 0; 0];
Q = C'*C;
Q_spec = blkdiag(Q, Q, Q);
B_spec = [zeros(4, 3); B zeros(4, 2); A*B B zeros(4, 1)];
R = 1/10;
R_spec = blkdiag(R, R, R);
A_spec = [eye(4); A; A^2];

H = 2*(B_spec'*Q_spec*B_spec + R_spec);
f = 2*x'*A_spec'*Q_spec*B_spec;
A_constraints = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
b_constraints = ones(6, 1);

%MPC controller
res = zeros(4, 41);
u_res = zeros(1, 41);
for k = 1:41
    f = 2*x'*A_spec'*Q_spec*B_spec;
    min_of_J = quadprog(H,f,A_constraints,b_constraints);
    u = min_of_J(1,1);
    x = A*x + B*u;
    res(:, k) = x;
    u_res(:, k) = u;
end

figure(2);
plot(res');

figure(3);
plot(u_res');