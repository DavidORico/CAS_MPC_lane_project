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
BM = []; % Expanded B-matrix
vq = [1,2,3]; q = diag(vq); vr = [1,2]; r = diag(vr);
Q = [];
R = [];
k1 = Linearisation(st, con,T );   % new
[x_next,a,b] = Linearisation(T/2*k1,con,T); %discretised state-space A/B
B0 = zeros(size(b));
BR =cell(1,N);
AM = eye(size(a)); % Expanded A-matrix
AN = eye(size(a));
BN = B0;
%% Realisation of the expanded matrices given N

for c = 0:N
Q = blkdiag(Q,q);
R = blkdiag(R,r);
c = c+1;
end

for k = 1:N % Creating the expanded A-matrix and initiliasing B
    if k == 1
        AM = AM;
    else
        AN = AN*a;
        AM = [AM;AN];
    end
    
    BR{N} = [];
    BR{1} = [BR{1},B0];
end

for k = 2:N %Creating the expanded B-matrix rows
    for c = 1:N %columns
        if k > c
            BR{k} = [BR{k}, a^(k-2-(c-1))*b];
        else
            BR{k} = [BR{k},B0];
        end
    end
end

for k = 1:N %Expanded B-matrix
    BM = [BM;BR{k}];
end
    
    
    
    
    
    



