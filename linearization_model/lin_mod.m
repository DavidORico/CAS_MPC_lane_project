%recreating the Linearization paper
%basic parameters
T = 0.1;
N = 5;
u_max = [0.47;
        3.77];
u_min = -u_max;
Q = diag([1 1 0.5]);
Q_spec = blkdiag(Q, Q, Q, Q, Q);
R = diag([0.1 0.1]);
R_spec = blkdiag(R, R, R, R, R);
%create reference trajectory
x = [-1 -1 0]';
num_reference_points=150;
ref_speed = zeros(1, num_reference_points) + 0.37;
ref_angle = sin(linspace(-pi, pi, num_reference_points));
ref_u = [ref_speed; ref_angle];
ref_x = zeros(3, num_reference_points);
for k = 1:num_reference_points
   A = [1 0 -ref_speed(k)*sin(ref_angle(k))*T;
        0 1 ref_speed(k)*cos(ref_angle(k))*T;
        0 0 1];
   B = [cos(ref_angle(k))*T 0;
        sin(ref_angle(k))*T 0;
        0 T];
   ref_x(:, k) = x;
   x = A*x + B*ref_u(:, k);
end
figure(1)
plot(ref_x(1,:), ref_x(2, :))

res_x = [];
res_u = [];
x = [-0.8 -0.7 0]';
for k = 1:num_reference_points-5
   %% update the A and B matrix
   A = [1 0 -ref_speed(k)*sin(ref_angle(k))*T;
        0 1 ref_speed(k)*cos(ref_angle(k))*T;
        0 0 1];
   B = [cos(ref_angle(k))*T 0;
        sin(ref_angle(k))*T 0;
        0 T];
   %% update the A and B matrices corresponding to the horizon 
   %A_spec = [A^(k-1);
   %           A^(k);
   %           A^(k+1);
   %           A^(k+2);
   %           A^(k+3)];
   A_spec = [eye(3);
             A;
             A^2;
             A^3;
             A^4];
   
   %b_row1 = [k-2, k-3, k-4, k-5, k-6];
   %b_row2 = b_row1 + 1;
   %b_row3 = b_row1 + 2;
   %b_row4 = b_row1 + 3;
   %b_row5 = b_row1 + 4;
   %B_spec = [b_row1;
   %          b_row2;
   %          b_row3;
   %          b_row4;
   %          b_row5];
   %B_dash = apply_power(A, B, B_spec);2];
   zr = zeros(3,2);
   B_dash = [zr, zr, zr, zr, zr;
             B, zr, zr, zr, zr;
             A*B, B, zr, zr, zr;
             A^2*B, A*B, B, zr, zr;
             A^3*B, A^2*B, A*B, B, zr];
   %% calculate u_min using quadprog
   H = 2*(B_dash'*Q_spec*B_dash + R_spec);
   f = 2*B_dash'*Q_spec*A_spec*(x-ref_x(:, k));
   constraints_l = zeros(N*2*2, N*2);
   constraints_l_p = [1 0; -1 0; 0 1; 0 -1];
   constraints_l(1:4, 1:2) = constraints_l_p;
   constraints_l(5:8, 3:4) = constraints_l_p;
   constraints_l(9:12, 5:6) = constraints_l_p;
   constraints_l(13:16, 7:8) = constraints_l_p;
   constraints_l(17:20, 9:10) = constraints_l_p;
   constraints_r = [0.47; 0.47; 3.77; 3.77];
   constraints_r = [constraints_r; constraints_r; constraints_r; constraints_r; constraints_r];
   u_quad = quadprog(H, f, constraints_l, constraints_r);
   u_min = u_quad(1:2, 1);
   %% go to next step
   x = A*x + B*u_min;
   res_x = [res_x, x];
   res_u = [res_u, u_min];
end
figure(2)
plot(res_x(1,:), res_x(2, :));
figure(3)
plot(res_u(1,:));
figure(4)
plot(res_u(2,:));

function B_completed = apply_power(A, B, mat)
    B_completed = [];
    sz = size(mat);
    for i = 1:sz(1)
        B_row = [];
        for j = 1:sz(2)
            B_row = [B_row, power_of_A_B(A, mat(i,j), B)];
        end
        B_completed = [B_completed; B_row];
    end
end

function A_dot = power_of_A_B(A, power, B)
    if power < 0
        A_dot = zeros(3, 2);
    elseif power == 0
        A_dot = B;
    else
        A_dot = (A^power)*B;
    end
end