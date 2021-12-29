clear all
%basic parameters
T = 0.04;
N = 5;
u_max_const = [40;
              pi/2-0.2];
u_min_const = [2;
               pi/2-0.2];
u_max_rate_change = [0.3; pi/4-0.2];
load testlap.mat;
my_test_lap = test_lap;
sz  = size(test_lap);
num_iterations = sz(1,1);

% get theta
theta = [];
for k = 1:num_iterations
    rotm = [test_lap(k, 5), test_lap(k, 6), test_lap(k, 7);
            test_lap(k, 8), test_lap(k, 9), test_lap(k, 10);
            test_lap(k, 11), test_lap(k, 12), test_lap(k, 13)];
    eulZYX = rotm2eul(rotm);
    theta = [theta, eulZYX(1,1)];
end

res_x = [];
res_u = [];
ref_x = [test_lap(:, 2)';
         test_lap(:, 3)';
         theta];
ref_u = [test_lap(:, 80)';
         test_lap(:, 14)'];
x = ref_x(:, 1);
u = ref_u(:, 1);
max_err_corrections = 20;
curr_err_corrections = 0;
treshold = 2;
k = 1;
while 1
   %% update the A and B matrix
   [A_spec,B_dash,Q_spec,R_spec,A,B] = statespace_Lin_MPC(ref_u(:,k),ref_x(:,k),N,T);
   %% calculate u_min using quadprog
   H = double(2*(B_dash'*Q_spec*B_dash + R_spec));
   f = double(2*B_dash'*Q_spec*A_spec*(x-ref_x(:,k)));
   [constraints_l,constraints_r] = get_constraints(N,u_max_const,u_min_const,u,u_max_rate_change);
   
   options = optimset('Display','off');
   u_quad = quadprog(H, f, constraints_l, constraints_r,[],[],[],[],[],options);
   u = u_quad(1:2, 1);
   %% go to next step
   x = A*x + B*u;
   res_x = [res_x, x];
   res_u = [res_u, u];
   %% check error
   err = norm(ref_x(:,k) - x,2);
   if k >= num_iterations
       break;
   elseif err < treshold || curr_err_corrections >= max_err_corrections
       k = k + 1;
       curr_err_corrections = 0;
   elseif err >= treshold
       curr_err_corrections = curr_err_corrections + 1;
   end
end
%modify the original path with the resulting MPC controlled path
% my_test_lap(:, 2:3) = res_x(1:2,:)';
% my_test_lap(:, 80) = res_u(1,:)';
% my_test_lap(:, 14) = res_u(2,:)';

%plotting the reference path and the path taken
figure(1);
ref_plot = plot(ref_x(1,:), ref_x(2, :));
title('Comparing Paths');
hold on;
res_plot = plot(res_x(1,:), res_x(2, :));
legend([ref_plot res_plot],{'reference path','result path'},'AutoUpdate','off')
% len = 2;
% end_points = [ref_x(1,:)+len*cos(ref_x(3,:));
%               ref_x(2,:)+len*sin(ref_x(3,:))];
% for k = 1:num_iterations
%     plot([ref_x(1,k) end_points(1,k)],[ref_x(2,k) end_points(2,k)])
% end
hold off;

%plotting inputs to the system
figure(2)
plot(res_u(1,:));
title('Inputs - u');
hold on;
plot(res_u(2,:));
hold off;
legend('velocity','steering angle');
%plot error
% figure(3);
% plot(abs(ref_x' - res_x'));
% title('Error');
% legend('err of x pos','error of y pos', 'error of heading');