
clear all; close all; clc;

% Links Length
l1 = 1; l2 = 0.5; l3 = 0.2;

% Desired position of end-effector
px = 0.5635;
py = -0.0493;
pz = -1.3485;

% Goal position
p_g = [ px py pz ]' ;

%%%%%%% Inverse Kinematics of a 4DOF manipulator - Jacobian method %%%%%%%
% Define joint angles limits
q_lower = (pi / 180) * [-10.0, 0.0, 0.0, 0.0]';
q_upper = (pi / 180) * [10.0, 90.0, 90.0, 90.0]';

% Error tolerance
tol = 10e-8;

% Initial guess
q = zeros(4, 1);

% Current end effector position (based on the initial guess)
p_c = forward_kinematics(q);

% Residual (dX)
res = p_g - p_c;

% Tolerance for jacobian residual (error)
jacob_tol = 10e-8;

% Number of iteration
iter_num = 0;

while( norm(res) > tol)

    % Update iterations nubmer
    iter_num = iter_num + 1;

    % Update jacobian 
    jacob = robot_jacobian(q);

    % Jacobian pseudo-inverse
    jacob_inv = pinv(jacob);

    % Jacobian error
    jacob_res = norm((eye(3) - jacob * jacob_inv) * res);
    
    while(jacob_res > jacob_tol) 
        % Update residual
        res = res / 2.0;

        % Udate jacobian error
        jacob_res = norm((eye(3) - jacob * jacob_inv) * res);
    end

    % Update robot state
    q = q + jacob_inv * res;

    % Check for limits (step 6)
    for i = 1: length(q)
        if (q(i) <= q_lower(i))
            q(i) = q_lower(i);
        elseif(q(i) >= q_upper(i))
            q(i) = q_upper(i);
        else
            continue;
        end
    end

    % Update the end effector position
    p_c = forward_kinematics(q);

    % Update residual
    res = p_g - p_c;
end


% Final q
disp(["Joint Angles: ") 
q * (180 / pi)

% End effector position
disp("End effector position: ") 
forward_kinematics(q)

% Error from desired position
disp("Error from desired position: ")
norm(res)

% Iterations number
disp("Number of iterations: ") 
iter_num


% Forward kinematics function
function y = forward_kinematics(q)
    % Links Length
    l1 = 1; l2 = 0.5; l3 = 0.2;

    % End effector position
    xt = cos(q(1)).*(l3*cos(q(2)+q(3)+q(4))+l2*cos(q(2)+q(3))+l1*cos((q(2)))) ;
    yt = sin(q(1)).*(l3*cos(q(2)+q(3)+q(4))+l2*cos(q(2)+q(3))+l1*cos(q(2)));
    zt = -(l3*sin(q(2)+q(3)+q(4))+l2*sin(q(2)+q(3))+l1*sin(q(2))) ;
    y = [xt, yt, zt]';
end


% Jacobian function 
function y = robot_jacobian(q)

    % Links Length
    l1 = 1; l2 = 0.5; l3 = 0.2;

    % Jacobian 
    y = zeros(3, 4);

    % First row
    y(1, 1) = -sin(q(1))*(l2*cos(q(2) + q(3)) + l1*cos(q(2)) + l3*cos(q(2) + q(3) + q(4)));
    y(1, 2) = -cos(q(1))*(l2*sin(q(2) + q(3)) + l1*sin(q(2)) + l3*sin(q(2) + q(3) + q(4)));
    y(1, 3) = -cos(q(1))*(l2*sin(q(2) + q(3)) + l3*sin(q(2) + q(3) + q(4)));
    y(1, 4) = -l3*sin(q(2) + q(3) + q(4))*cos(q(1));

    % Second row
    y(2, 1) = cos(q(1))*(l2*cos(q(2) + q(3)) + l1*cos(q(2)) + l3*cos(q(2) + q(3) + q(4)));
    y(2, 2) = -sin(q(1))*(l2*sin(q(2) + q(3)) + l1*sin(q(2)) + l3*sin(q(2) + q(3) + q(4)));
    y(2, 3) = -sin(q(1))*(l2*sin(q(2) + q(3)) + l3*sin(q(2) + q(3) + q(4)));
    y(2, 4) = -l3*sin(q(2) + q(3) + q(4))*sin(q(1));

    % Third row
     y(3, 1) = 0.0;
     y(3, 2) = - l2*cos(q(2) + q(3)) - l1*cos(q(2)) - l3*cos(q(2) + q(3) + q(4));
     y(3, 3) = - l2*cos(q(2) + q(3)) - l3*cos(q(2) + q(3) + q(4));
     y(3, 4) = -l3*cos(q(2) + q(3) + q(4));
end

