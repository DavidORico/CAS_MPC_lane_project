%Defining the reference tracking linearised RHS by Taylor expansion around
%reference
function [A,B] = Linearisation(ref_x, ref_u,k,T)
syms v; syms delta;
syms theta; syms x; syms y;
L = 2;
%% Jacobian of discrete kinematics
% z = [x+v*cos(theta)*T;y+v*sin(theta)*T;theta+(v/L)*tan(delta)*T];
% A = jacobian(z,[x,y,theta]);
% B = jacobian(z,[v,delta]);
% A(1,3) = (-ref_u(1,k))*sin(ref_x(3, k));
% A(2,3) = (ref_u(1,k))*cos(ref_x(3, k));
% B(1,1) = cos(ref_x(3, k));
% B(2,1) = sin(ref_x(3, k));
% B(3,1) = (tan(ref_u(2,k))/2);
% B(3,2) = ((ref_u(1,k))*(tan(ref_u(2,k))^2)+1)/2;
A = [1 0 (-ref_u(1,k))*sin(ref_x(3, k))*T;
     0 1 (ref_u(1,k))*cos(ref_x(3, k))*T;
     0 0 1];
B = [cos(ref_x(3, k))*T 0;
     sin(ref_x(3, k))*T 0;
     (tan(ref_u(2,k))/L)*T (((ref_u(1,k))*(tan(ref_u(2,k))^2)+1)/L)*T];
end