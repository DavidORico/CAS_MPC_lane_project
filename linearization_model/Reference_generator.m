function [ref_x_outer,ref_u,curvature_radius] = Reference_generator(global_xy, T)
%Reference_generator (theta,v, delta)
L = 2.5;
Lfw = 3;    %corresponds to looking distance from the trajectory
WL = 4; %Width of a lane;
theta = [];
[x_inner, y_inner, x_outer, y_outer, curvature_radius] = parallel_curve(global_xy(1,:), global_xy(2,:), WL/2,0,0);
[b,a] = size(x_inner);
ref_x_inner = [x_inner';y_inner'];
ref_x_outer = [x_outer';y_outer'];
ref_u = zeros(a,b);

%% Local angles
for k=1:b
    if k < b
       theta = [theta, acos(dot(ref_x_outer(:,k),ref_x_outer(:,k+1))/(norm(ref_x_outer(:,k))*norm(ref_x_outer(:,k+1))))];
    else
        theta = [theta,0];
    end
end
ref_x_outer = real([ref_x_outer;theta]);
%% Steering angle
for k = 1:b
    if k < b
        delta = -atan((L*sin(ref_x_outer(3,k+1)))/((Lfw/2)+Lfw*cos(ref_x_outer(3,k+1))));
        ref_u(2,k) = real(delta);
    else
        delta = 0;
    end
end
%% Reference velocity
Vr = 5;
%P-controller
Kp = 0.005;
for k = 1:b
    if k < b
        distancetonext = norm(ref_x_outer(:,k+1)-ref_x_outer(:,k),2);
    else
        distancetonext = 0;
    end
    velocity = distancetonext/T;
    ref_u(1,k) = real(abs(velocity-(ref_u(1,k)))*Kp+Vr);
end
end
        
 


