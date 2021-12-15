%Defining the reference tracking linearised RHS by Taylor expansion around
%reference
function [model,A,B] = Linearisation(states,controls)
syms v; syms delta;
syms theta; syms x; syms y;
L = 2;
reference_states = [1;1;1];
reference_controls = [1;1];
x_error = states - reference_states;
u_error = controls - reference_controls;

%% Jacobian of discrete kinematics
z = [x+v*cos(theta);y+v*sin(theta);theta+(v/L)*tan(delta)];
A = jacobian(z,[x,y,theta]);
B = jacobian(z,[v,delta]);
A(1,3) = (-controls(1,1))*sin(states(3,1));
A(2,3) = (controls(1,1))*cos(states(3,1));
B(1,1) = cos(states(3,1));
B(2,1) = sin(states(3,1));
B(3,1) = (tan(controls(2,1))/2);
B(3,2) = ((controls(1,1))*(tan(controls(2,1))^2)+1)/2;
%% 
rhs = [controls(1,1)*cos(states(3,1));controls(1,1)*sin(states(3,1));...
    (controls(1,1)/L)*tan(controls(2,1))];
RHS = rhs + A*x_error + B*u_error; %Fully defined RHS
model = RHS -rhs; %Reference tracking kinematics model
A = A;
B = B;
end