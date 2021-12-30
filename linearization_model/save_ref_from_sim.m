clear all
%basic parameters
T = 0.04;
N = 20;
u_max_const = [40;
              pi/2-0.2];
u_min_const = [2;
               pi/2-0.2];
u_max_rate_change = [0.3; pi/4-0.2];
%u_max_rate_change = u_max_const;
load testlap.mat;
sz  = size(test_lap);
num_iterations = sz(1,1);

res_x = [];
res_u = [];
middle_road_ref = [test_lap(:, 2)';test_lap(:, 3)'];
[ref_x, ref_u] = Reference_generator([test_lap(:, 2)';test_lap(:, 3)'], T);
for k = 1:50
    ref_x(1:2,k) = [test_lap(1, 2);test_lap(1, 3)];
    ref_u(:,k) = [test_lap(1,80);test_lap(1,14)];
end
ref_x(:,1) = [test_lap(1,2);test_lap(1,3);0];
ref_u(:,1) = [5;test_lap(1,14)];
x = ref_x(:, 1);
u = ref_u(:, 1);
max_err_corrections = 20;
curr_err_corrections = 0;
treshold = 2;
k = 1;
my_test_lap = [];
while 1
   %% update the A and B matrix
   [A_spec,B_dash,Q_spec,R_spec,A,B] = statespace_Lin_MPC(ref_u(:,k),ref_x(:,k),N,T);
   %% calculate u_min using quadprog
   H = double(2*(B_dash'*Q_spec*B_dash + R_spec));
   f = double(2*B_dash'*Q_spec*A_spec*(x-ref_x(:,k)));
   if k == 1
       [constraints_l,constraints_r] = get_constraints(N,u_max_const,u_min_const,u,u_max_rate_change,x,middle_road_ref(:,k),middle_road_ref(:,k+1));
   else
       [constraints_l,constraints_r] = get_constraints(N,u_max_const,u_min_const,u,u_max_rate_change,x,middle_road_ref(:,k-1),middle_road_ref(:,k));
   end
   options = optimset('Display','off');
   u_quad = quadprog(H, f, constraints_l, constraints_r,[],[],[],[],[],options);
   u = u_quad(1:2, 1);
   %% save to file for simulation
   my_test_lap = [my_test_lap; test_lap(k,:)];
   my_test_lap(end,2:3) = x(1:2,:)';
   my_test_lap(end,80) = u(1,:);
   my_test_lap(end,14) = u(2,:);
   %% go to next step
   x = A*x + B*u;
   res_x = [res_x, x];
   res_u = [res_u, u];
   %% check error
   err = norm(ref_x(:,k) - x,2);
   if k >= num_iterations
       break;
   elseif err < treshold || curr_err_corrections >= max_err_corrections
       k = k + 1
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

save('my_test_lap.mat','my_test_lap');