%Transformation matrices
syms q1 q2 q3 q4 l1 l2 l3 'real'

T1 = [cos(q1), 0, -sin(q1), 0.0; sin(q1), 0.0, cos(q1), 0.0; ...
    0.0, -1.0 0.0 0.0; 0.0, 0.0, 0.0, 1.0];


T2 = [cos(q2), -sin(q2), 0.0, l1*cos(q2); sin(q2), cos(q2) 0.0, l1*sin(q2); ...
    0.0, 0.0, 1.0, 0.0; 0.0, 0.0, 0.0, 1.0];

T3 = [cos(q3), -sin(q3), 0.0, l2*cos(q3); sin(q3), cos(q3), 0.0, l2*sin(q3); ...
    0.0, 0.0, 1.0, 0.0; 0.0, 0.0, 0.0, 1.0];

T4 = [cos(q4), -sin(q4), 0.0, l3*cos(q4); sin(q4), cos(q4), 0.0, l3*sin(q4); ...
    0.0, 0.0, 1.0, 0.0; 0.0, 0.0, 0.0, 1.0];

% Total transformation matrix
Tf = T1 * T2 * T3 * T4;

% Get postion
xe = Tf(1, 4);
ye = Tf(2, 4);
ze = Tf(3, 4);