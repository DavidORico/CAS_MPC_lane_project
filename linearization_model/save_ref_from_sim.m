clear all
%basic parameters
T = 0.04;
N = 20;
u_max_const = [40;
              pi/2-0.2];
u_min_const = [2;
               pi/2-0.2];

load octavia_testlap.mat;
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
tic
for k = 1:num_iterations
   %% update the A and B matrix
   [A_spec,B_dash,Q_spec,R_spec,A,B] = statespace_Lin_MPC(ref_u(:,k),ref_x(:,k),1,N,T);
   %% calculate u_min using quadprog
   H = double(2*(B_dash'*Q_spec*B_dash + R_spec));
   f = double(2*B_dash'*Q_spec*A_spec*(x-ref_x(:,k)));
%    if N+k < num_iterations
%        f = double(2*B_dash'*Q_spec*A_spec*(x-ref_x(:,N+k)));
%    else
%        f = double(2*B_dash'*Q_spec*A_spec*(x-ref_x(:,k)));
%    end
   
   constraints_l = zeros(N*2*2, N*2);
   constraints_l_p = [1 0; -1 0; 0 1; 0 -1];
   for c = 1:4:N*4-3
       constraints_l(c:c+3, ((c-1)/4)*2+1:((c-1)/4)*2+2) = constraints_l_p;
   end
   constraints_r = [u_max_const(1, 1); u_min_const(1, 1); u_max_const(2, 1); u_min_const(2, 1)];
   final_r = [];
   for c =1:N
       final_r = [final_r ; constraints_r];
   end
   u_quad = quadprog(H, f, constraints_l, final_r);
   u_min = u_quad(1:2, 1);
   %% go to next step
   x = A*x + B*u_min;
   res_x = [res_x, x];
   res_u = [res_u, u_min];
end
toc

%modify the original path with the resulting MPC controlled path
my_test_lap(:, 2:3) = res_x(1:2,:)';
my_test_lap(:, 80) = res_u(1,:)';
my_test_lap(:, 14) = res_u(2,:)';

%plotting the reference path and the path taken
figure(1);
plot(ref_x(1,:), ref_x(2, :));
title('Comparing Paths');
hold on;
plot(res_x(1,:), res_x(2, :));
hold off;
legend('reference path','result path');
figure(2)
plot(res_u(1,:));
title('Inputs - u');
hold on;
plot(res_u(2,:));
hold off;
legend('velocity','steering angle');
%plot error
figure(3);
plot(abs(ref_x' - res_x'));
title('Error');
legend('err of x pos','error of y pos', 'error of heading');