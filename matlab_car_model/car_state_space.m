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

A_new = [-(2*Cf+2*Cr)/(m*Vx), 0, -Vx-(2*Cf*lf+2*Cr*lr)/(m*Vx);
    0, 0, 1;
    -(2*Cf*lf+2*Cr*lr)/(Iz*Vx), 0,-(2*Cf*lf*lf+2*Cr*lr*lr)/(Iz*Vx) ];
B_new = [(2*Cf)/m; 0; (2*lf*Cf)/Iz];
C_new = [0 1 0];
