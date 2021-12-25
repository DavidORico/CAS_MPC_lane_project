function [new_ref_x, new_ref_u] = generate_reference(type_of_ref,prev_ref_x,prev_ref_u,T)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
new_ref_x = [];
if strcmp(type_of_ref, 'straight')
    new_ref_u = [prev_ref_u(1,1); 0];
    [A,B] = Linearisation(prev_ref_x, new_ref_u,1,T);
    new_ref_x = A*prev_ref_x + B*new_ref_u;
elseif strcmp(type_of_ref, 'left')
    new_ref_u = [prev_ref_u(1,1); -pi/6];
    [A,B] = Linearisation(prev_ref_x, new_ref_u,1,T);
    new_ref_x = A*prev_ref_x + B*new_ref_u;
elseif strcmp(type_of_ref, 'right')
    new_ref_u = [prev_ref_u(1,1); pi/6];
    [A,B] = Linearisation(prev_ref_x, new_ref_u,1,T);
    new_ref_x = A*prev_ref_x + B*new_ref_u;
end
end

