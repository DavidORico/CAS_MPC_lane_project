load octavia_testlap.mat;
% ref_u = [test_lap(:, 80)';
%          test_lap(:, 14)'];
ref_u = [zeros(1,2790)+1;
        test_lap(:, 14)'];
x = [0; 0; 0];
T = 0.04;
num_reference_points = 2790;

ref_x = zeros(3, num_reference_points);
for k = 1:num_reference_points
   [A,B] = Linearisation(x, ref_u(:, k),1,T);
   %A = [1 0 -ref_speed(k)*sin(ref_angle(k))*T;
   %     0 1 ref_speed(k)*cos(ref_angle(k))*T;
   %     0 0 1];
   %B = [cos(ref_angle(k))*T 0;
   %     sin(ref_angle(k))*T 0;
   %     0 T];
   ref_x(:, k) = x;
   x = A*x + B*ref_u(:, k);
end

figure(1);
ref_plot = plot(ref_x(1,:), ref_x(2, :));