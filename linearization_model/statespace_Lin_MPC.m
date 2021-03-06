function [AM,BM,Q,R,a,b] = statespace_Lin_MPC(ref_u,ref_x,N,T)
%Main function
%% Euler discretisation and MPC-matrices

a = []; b = [];
BM = []; % Expanded B-matrix
vq = [1 1 0.5]; q = diag(vq); vr = [0.1 0.1]; r = diag(vr);
Q = [];
R = [];
[a,b] = Linearisation(ref_x, ref_u,T); %discretised state-space A/B
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
    
    
    
    
    
    



