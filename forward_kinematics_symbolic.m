clear all; close all;

syms q1 q2 q3 q4 'real'

syms l1 l2 l3 'real'




%**************** DH parameters ****************

% Link 1
dh1_params = [0.0, pi/2, 0.0, q1]';

% Link 2
dh2_params = [l1, 0.0, 0.0, q2]';

% Link 3 
dh3_params = [l2, 0.0, 0.0, q3]';

% Link 4
dh4_params = [l3, 0.0, 0.0, q4]';


% Relative transformations

T01 = transformation_mat(dh1_params);

T12 = transformation_mat(dh2_params);

T23 = transformation_mat(dh3_params);

T34 = transformation_mat(dh4_params);

% Total transformation
T04 = T01 * T12 * T23 * T34;

xt = simplify(T04(1, end));
yt = simplify(T04(2, end));
zt = simplify(T04(3, end));




function A = transformation_mat(dh_params)
    % Returns the trasformation matrix of Frame i 
    % with respect to Frame i-1.
    % dh_params = [a_i, alpha_i, d_i, theta_i]
            
    ai = dh_params(1); alphai = dh_params(2);
    di = dh_params(3); thetai = dh_params(4); 

    A = [cos(thetai), -cos(alphai)*sin(thetai), sin(alphai)*sin(thetai), ...
        ai*cos(thetai); sin(thetai), cos(alphai)*cos(thetai), ...
        -sin(alphai)*cos(thetai), ai*sin(thetai); 0, sin(alphai), ...
        cos(alphai), di; 0, 0, 0, 1];
end