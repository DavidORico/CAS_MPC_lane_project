%recreating the Linearization paper
clear all
%basic parameters
T = 0.1;
N = 20;
u_max_const = [12; pi/2];
u_min_const = u_max_const;
u_max_rate_change = [0.5; 0.2];
num_iterations = 210;
type_of_ref = 'sin';

%create reference trajectory
x = [-1 -1 0]';
ref_u = [];
ref_x = [];
global_xy = [];
if strcmp(type_of_ref, 'spiral')
    %Euler spiral
    load euler_spiral_xy.mat data_xy
    global_xy = data_xy;
    global_xy(:,1) = [0.001; 0.001];
    num_iterations = 1001;
elseif strcmp(type_of_ref, 'turn')
    % Sharp turn
    global_xy = [1:num_iterations/2, zeros(1,num_iterations/2)+num_iterations/2;
                 zeros(1,num_iterations/2), 1:num_iterations/2];
elseif strcmp(type_of_ref, 'sin')
    % Wave
    global_xy = [1:num_iterations;
                sin(linspace(-pi, pi, num_iterations))];
elseif strcmp(type_of_ref, 'straight')
    % straight path
    global_xy = [1:num_iterations;
                1:num_iterations];
end

[ref_x, ref_u] = Reference_generator(global_xy,T);

res_x = [];
res_u = [];
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
   [constraints_l,constraints_r] = get_constraints_simple(N,u_max_const,u_min_const,u,u_max_rate_change);
   
   options = optimset('Display','off');
   u_quad = quadprog(H, f, constraints_l, constraints_r,[],[],[],[],[],options);
   %u_quad = quadprog(H, f, constraints_l, constraints_r);
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
