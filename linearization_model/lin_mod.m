%recreating the Linearization paper
%basic parameters
T = 0.1;
N = 5;
u_max = [0.47;
        3.77];
u_min = -u_max;
Q = diag([1 1 0.5]);
Q_spec = blkdiag(Q, Q, Q, Q, Q);
R = [0 1; 0 1];
R_spec = blkdiag(R, R, R, R, R);
x0 = [-1 -1 0]';
num_reference_points=150;
ref_speed = zeros(1, num_reference_points) + 0.37;
ref_angle = sin(linspace(-pi, pi, num_reference_points));

for k = 1:num_reference_points-5
   A = [1 0 -ref_speed(k)*sin(ref_angle(k))*T;
        0 1 ref_speed(k)*cos(ref_angle(k))*T;
        0 0 1];
    B = [cos(ref_angle(k))*T 0;
        sin(ref_angle(k))*T 0;
        0 T];
    
    A_spec = [A^(k-1);
              A^(k);
              A^(k+1);
              A^(k+2);
              A^(k+3)];
end

function a = alpha_A(mat_given, k, j, l, N)
% MYMEAN Local function that calculates mean of array.
    i = N-j
    a = eye(size(mat_given, 1))
    if i < l
        for m = i:l
           a =  
        end
    else
        for m = l:i
            
        end
    end
end