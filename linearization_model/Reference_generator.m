function [ref_x,ref_u] = Reference_generator(global_xy, T)
%Reference_generator (theta,v, delta)
L = 2;
Lfw = 2;    %corresponds to distance from the trajectory
theta = [];
[a,b] = size(global_xy);
ref_x = [];
ref_u = zeros(a,b);
%required: Local angles and steering angle/rate
%% Local angles
for k=1:b
    if k < b
       theta = [theta, acos(dot(global_xy(:,k),global_xy(:,k+1))/(norm(global_xy(:,k))*norm(global_xy(:,k+1))))];
    else
        theta = [theta,0];
    end
end
ref_x = real([global_xy;theta]);
%% Steering angle
for k = 1:b
    if k < b
        delta = -atan((L*sin(ref_x(3,k+1)))/((Lfw/2)+Lfw*cos(ref_x(3,k+1))));
        ref_u(2,k) = real(delta);
    else
        delta = 0;
    end
end
%% Reference velocity
Vr = 1;
%P-controller
Kp = 0.005;
for k = 1:b
    if k < b
        distancetonext = norm(global_xy(:,k+1)-global_xy(:,k),2);
    else
        distancetonext = 0;
    end
    velocity = distancetonext/T;
    ref_u(1,k) = real(abs(velocity-(ref_u(1,k)))*Kp+Vr);
end
end
        
 


