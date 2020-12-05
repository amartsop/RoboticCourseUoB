clear all; close all;
% Define joint variables and lengths
syms q1 q2 q3 q4 'real'
syms l1 l2 l3 l4 'real'

% End effector position
xt = cos(q1).*(l3*cos(q2+q3+q4)+l2*cos(q2+q3)+l1*cos(q2)) ;
yt = sin(q1).*(l3*cos(q2+q3+q4)+l2*cos(q2+q3)+l1*cos(q2));
zt = -(l3*sin(q2+q3+q4)+l2*sin(q2+q3)+l1*sin(q2)) ;

p = [xt, yt, zt]';

% State vector
q = [q1, q2, q3, q4]';

% Jacobian dp/dq
J = jacobian(p, q);

% You can either just print and copy the values or create a jacobian function
% using the command "matlabFunction"
% https://uk.mathworks.com/help/symbolic/matlabfunction.html


%% First row
% J11 = -sin(q1)*(l2*cos(q2 + q3) + l1*cos(q2) + l3*cos(q2 + q3 + q4));
% J12 = -cos(q1)*(l2*sin(q2 + q3) + l1*sin(q2) + l3*sin(q2 + q3 + q4));
% J13 = -cos(q1)*(l2*sin(q2 + q3) + l3*sin(q2 + q3 + q4));
% J14 = -l3*sin(q2 + q3 + q4)*cos(q1);

%% Second row
% J21 = cos(q1)*(l2*cos(q2 + q3) + l1*cos(q2) + l3*cos(q2 + q3 + q4));
% J22 = -sin(q1)*(l2*sin(q2 + q3) + l1*sin(q2) + l3*sin(q2 + q3 + q4));
% J23 = -sin(q1)*(l2*sin(q2 + q3) + l3*sin(q2 + q3 + q4));
% J24 = -l3*sin(q2 + q3 + q4)*sin(q1);

%% Third row
% J31 = 0.0;
% J32 = - l2*cos(q2 + q3) - l1*cos(q2) - l3*cos(q2 + q3 + q4)
% J33 = - l2*cos(q2 + q3) - l3*cos(q2 + q3 + q4);
% J34 = -l3*cos(q2 + q3 + q4);