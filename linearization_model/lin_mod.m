%recreating the Linearization paper
clear all
%basic parameters
T = 0.1;
N = 5;
u_max_const = [4;
         pi/4];
u_min_const = u_max_const;
num_iterations = 210;
type_of_ref = 'wave';

%create reference trajectory
x = [-1 -1 0]';
ref_u = [];
ref_x = [];

if strcmp(type_of_ref, 'spiral')
    % Euler spiral
    ref_speed = zeros(1, num_iterations) + 1.0;
    ref_angle = sin(linspace(-pi, pi, num_iterations));
    ref_u = [ref_speed; ref_angle];
    ref_x = zeros(3, num_iterations);
    for k = 1:num_iterations
       [A,B] = Linearisation(x, ref_u(:, k),T);
       ref_x(:, k) = x;
       x = A*x + B*ref_u(:, k);
    end
elseif strcmp(type_of_ref, 'turn')
    % Sharp turn
    ref_speed = zeros(1, num_iterations) + 1.0;
    ref_angle = [zeros(1, num_iterations/3), zeros(1, num_iterations/3)+(pi/5), zeros(1, num_iterations/3)];
    ref_u = [ref_speed; ref_angle];
    ref_x = zeros(3, num_iterations);
    for k = 1:num_iterations
       [A,B] = Linearisation(x, ref_u(:, k),T);
       ref_x(:, k) = x;
       x = A*x + B*ref_u(:, k);
    end
elseif strcmp(type_of_ref, 'wave')
    % Wave
    ref_speed = zeros(1, num_iterations) + 1.0;
    ref_angle = linspace(-pi/4, pi/4, num_iterations);
    ref_u = [ref_speed; ref_angle];
    ref_x = zeros(3, num_iterations);
    for k = 1:num_iterations
       [A,B] = Linearisation(x, ref_u(:, k),T);
       ref_x(:, k) = x;
       x = A*x + B*ref_u(:, k);
    end
end

res_x = [];
res_u = [];
x = ref_x(:, 1);

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
   [constraints_l,constraints_r] = get_constraints(N,u_max_const,u_min_const);
   
   options = optimset('Display','off');
   u_quad = quadprog(H, f, constraints_l, constraints_r,[],[],[],[],[],options);
   u_min = u_quad(1:2, 1);
   %% go to next step
   x = A*x + B*u_min;
   res_x = [res_x, x];
   res_u = [res_u, u_min];
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
% figure(3);
% plot(abs(ref_x' - res_x'));
% title('Error');
% legend('err of x pos','error of y pos', 'error of heading');
