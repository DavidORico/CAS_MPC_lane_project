function [con_l,con_r] = get_constraints(N,u_max,u_min)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
sz_u = numel(u_max);
con_l = [eye(N*sz_u);
         -eye(N*sz_u)];
con_r = [repmat(u_max,N,1);
         repmat(u_min,N,1)];
end

