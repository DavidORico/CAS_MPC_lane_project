function [con_l,con_r] = get_constraints(N,u_max,u_min,prev_u,max_rate_change_u,x,road_mid_p,road_mid_n)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
sz_u = numel(u_max);
new_u_max = prev_u + max_rate_change_u;
new_u_min = -(prev_u - max_rate_change_u);
for k = 1:sz_u
    if new_u_max(k,1) > u_max(k,1)
        new_u_max(k,1) = u_max(k,1);
    end
    if new_u_min(k,1) > u_min(k,1)
        new_u_min(k,1) = u_min(k,1);
    end
end
con_l = [eye(N*sz_u);
         -eye(N*sz_u)];
con_r = [repmat(new_u_max,N,1);
         repmat(new_u_min,N,1)];
new_ref = (road_mid_p + road_mid_n) / 2;
radius = norm(new_ref - road_mid_n,2);
if norm(x(1:2,:) - new_ref,2) < radius/5
    %if -new_u_min(2,1) < 0 && new_u_max(2,1) > 0
        con_r(2,1) = 0;
    %end
end
% if norm(x(1:2,:) - new_ref,2) >= 2 + radius
%     %if -new_u_min(2,1) < 0 && new_u_max(2,1) > 0
%         con_r(N*sz_u + 2,1) = 0;
%     %end
% end
end

