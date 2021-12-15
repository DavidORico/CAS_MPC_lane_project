clc
clear all
%Main function
%% Initialising variables
sympref('FloatingPointOutput',true)
syms v; syms delta; syms theta;
syms x;syms y;
N =2;
T = 0.1;
L = 2;
%% Jacobians
states = [x; y; theta];
controls = [v;delta];
initial_states = [1;1;1];
initial_controls = [2;2];
X(1:3,1) = initial_states;
z = [v*cos(theta);v*sin(theta);(v/L)*tan(delta)];
A = jacobian(z,[x,y,theta]); %Continuous-time state-space
B = jacobian(z,[v,delta]); %Continuous-time state-space
%% Euler discretisation and MPC-matrices
st = initial_states; a = []; b = [];
con = initial_controls;
X = [];
AM = eye(3); % Expanded A-matrix
AN = eye(3);
BM = zeros(N,N); % Expanded B-matrix
vq = [1,2,3]; q = diag(vq); vr = [1,2]; r = diag(vr);
Q = [];
R = [];
for c = 0:N
Q = blkdiag(Q,q);
R = blkdiag(R,r);
c = c+1;
end
for k = 1:N
    k1 = Linearisation(st, con);   % new
    k2 = st + Linearisation(T/2*k1, con); % new
    [x_next,a,b] = Linearisation(T/2*k1,con); %discretised state-space A/B
    X((3*k)+1:(3*(k+1)),1) = x_next;
    st = x_next;
    AN = AN*a;
    AM = [AM;AN];
end
%% Quadprog formulation





