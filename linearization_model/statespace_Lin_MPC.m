function [AM,BM,Q,R,a,b] = statespace_Lin_MPC(ref_u,ref_x,k,N,T)
%Main function
%% Initialising variables
sympref('FloatingPointOutput',true)
syms v; syms delta; syms theta;
syms x;syms y;
L = 2;

%% Jacobians

states = [x; y; theta];
controls = [v;delta];
z = [v*cos(theta);v*sin(theta);(v/L)*tan(delta)];
A = jacobian(z,[x,y,theta]); %Continuous-time state-space
B = jacobian(z,[v,delta]); %Continuous-time state-space

%% Euler discretisation and MPC-matrices

a = []; b = [];
BM = []; % Expanded B-matrix
vq = [1 1 0.5]; q = diag(vq); vr = [0.1 0.1]; r = diag(vr);
Q = [];
R = [];
[a,b] = Linearisation(ref_x, ref_u,k,T); %discretised state-space A/B
B0 = zeros(size(b));
BR =cell(1,N);
AM = eye(size(a)); % Expanded A-matrix
AN = eye(size(a));
BN = B0;
%% Realisation of the expanded matrices given N

for c = 1:N
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
    
    
    
    
    
    



