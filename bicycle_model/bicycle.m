clear all
% System states
% x1 = beta (vehicle slip angle [rad]) 
% x2 = r (yaw rate [rad/s])
 
% Input
% u1 = delta (steer angle [rad])
delta_deg = 10;
 
% Outputs
% y1 = v  (sideslip velocity [m/s])
% y2 = r  (yaw rate [rad/s])
% y3 = ay (lateral acceleration [m/s^2]) 
 
% Boundary condition
V = 10;    % forward velocity [m/s]
 
% Initial system states
x1_0 = 0; % beta at t = 0 [rad]
x2_0 = 0; % r    at t = 0 [rad/s]
 
 
% Vehicle parameters
Cf = -1020; % front tyre cornering stiffness per tyre [N/deg]
Cr = -760;  % rear tyre cornering stiffness per tyre [N/deg] 
mf = 620;  % mass at front axle [kg]
mr = 430;  % mass at rear axle [kg]
Iz = 1560; % yaw moment of inertia [kg m^2]
l  = 2.4;  % wheelbase [m]
 
% Conversion of units
Cf2 = Cf*2*180/pi; % front cornering siffness per axle [N/rad]
Cr2 = Cr*2*180/pi; % front cornering siffness per axle [N/rad]
m = mf + mr;       % total mass of vehicle [kg]
a = l * mr/m;      % distance between c.g. and front axle [m]
b = l * mf/m;      % distance between c.g. and rear axle [m]
 
% Bicycle model derivatives
Ybeta  = Cf2 + Cr2;
Yr     = (a*Cf2 - b*Cr2)/V;
Ydelta = -Cf2;
Nbeta  = a*Cf2 - b*Cr2;
Nr     = (a^2*Cf2 + b^2*Cr2)/V;
Ndelta = -a*Cf2;
 
% State-space matrices
A = [Ybeta/(m*V), (Yr/(m*V) - 1)
    Nbeta/Iz, Nr/Iz];
 
B = [Ydelta/(m*V)
    Ndelta/Iz];
 
C = [V, 0
    0, 1
    Ybeta/m, Yr/m];
 
D = [0
    0
    Ydelta/m];