%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file provides code for numerical estimation of the jacobian 
% of a function f(x) around a point x0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%% Example script %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Point of evaluation
x0 = [0.0, 1.0, 1.0]';

% Implementation of numerical jacobian around point x
dfdx(x0)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% // Numerical estimation of system's jacobian
function y = dfdx(x)
    % Evaluation of f at point x 
    fx = f(x);

    % Perturbation variable
    x_pert = x;

    % Initialize jacobian 
    jac = zeros(size(fx, 1), size(x, 1));

    % Approximation tolerance  
    tol = 10e-10;

    % Iterate and approximate jacobian
    for i = 1: size(x, 1)
        % Perturbate x 
        x_pert(i) = x_pert(i) + tol;

        % Finite difference derivative approximation
        jac(:, i) = (f(x_pert) - fx) / tol;

        x_pert(i) = x(i);
    end
    y = jac;
end


% Example function 
function y = f(x)
    % y1 = x1 
    % y2 = 5 * x3
    % y3 = 4 * x2^2 - 2 * x3
    % y4 = x3 * sin(x1)

    y1 = x(1);
    y2 = 5.0 * x(3);
    y3 = 4.0 * x(2)^(2.0) - 2.0 * x(3);
    y4 = x(3) * sin(x(1));

    y = [y1, y2, y3, y4]';
end
