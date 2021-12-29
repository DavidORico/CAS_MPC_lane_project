%Reference_generator (theta,v, delta)
ref_xy = [1 4 3 -5 -3 5; -3 4 2 -3.2 -7 10];
T = 0.04;
L = 2;
Lfw = 1;    %corresponds to distance from the trajectory
theta = [];
[a,b] = size(ref_xy);
ref_x = [];
ref_u = zeros(a,b);
%required: Local angles and steering angle/rate
%% Local angles
for k=1:b
    if k < b
       theta = [theta, acos(dot(ref_xy(:,k),ref_xy(:,k+1))/(norm(ref_xy(:,k))*norm(ref_xy(:,k+1))))];
    else
        theta = [theta,0];
    end
end
ref_x = [ref_xy;theta];
%% Steering angle
for k = 1:b
    if k < b
        delta = -atan((L*sin(ref_x(3,k+1)))/((Lfw/2)+Lfw*cos(ref_x(3,k+1))));
        ref_u(2,k) = delta;
    else
        delta = 0;
    end
end
%% Reference velocity
Vr = 10;
%P-controller
Kp = 0.05;
for k = 1:b
    if k < b
        distancetonext = norm(ref_xy(:,k+1)-ref_xy(:,k),2);
    else
        distancetonext = 0;
    end
    velocity = distancetonext/T;
    ref_u(1,k) = abs(velocity-(ref_u(1,k)))*Kp+Vr;
end
        
 


