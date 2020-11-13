%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file provides a pseudocode for an efficient calculation of the workspace
% for your robotic manipulators (index and thumb). This algorithm can 
% substitute the use of nested for loops.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Let's study the case of the thumb. We know that it must have 5 degrees of 
% freedom (DOF) q = [theta1, theta2, ..., theta5]'. Each DOF is defined inside 
% a range of values, such that:
%              thetai = [thetai_min, thetai_max] 

% Also, we assume that you have defined function of the end effector position
% as:

% function y = end_effector_position(q)
%     % From the expressions of your final transformamtion matrix
%     xe = ... 
%     ye = ...
%     ze = ...
%     y = [xe, ye, ze];
% end

% Then you can easily calculat and plot your workspace using the following
% commands

% traj = workspace_trajectory();
% y = end_effector_position(traj); 
% scatter3( y(:, 1), y(:, 2), y(:, 3), '*');
% xlabel('x'); 
% ylabel('y');  
% zlabel('z');       


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The function "workspace_trajectory()" creates a grid that includes 
% all the possible combinations of [theta1, theta2, ... theta5] inside their 
% range. The same result can be obtained by iterating through the values of 
% each thetai, but this is a bit computationaly expensive.

function y = workspace_trajectory()

    % "Steps" defines the number of discretizations. Is chosen the same for all 
    % thetai (i = 1, 2, 3, 4, 5). The bigger the step the higher the accuracy.
    steps = 20;         

    % q is a 5 dimensional array in the case of the thumb model
    dimensions = 5; groups = dimensions;


    % Define the range of your joint angles thetai_min and thetai_max here.
    % These values depend on your problem. Try not to brake any thumbs!

    % Then caclulcate the linspace array based on the steps specified above.
    % Not the most efficient way of doing it but this is a pseudocode.
    theta1 = linspace(theta1_min, theta1_max, steps)';
    theta2 = linspace(theta2_min, theta2_max, steps)';
    theta3 = linspace(theta3_min, theta3_max, steps)';
    theta4 = linspace(theta4_min, theta4_max, steps)';
    theta5 = linspace(theta5_min, theta5_max, steps)';

   
    % This is the essence of the code. It creates the n-dimensional grid 
    % (5 in this case)
    [THETA5, THETA4, THETA3, THETA2, THETA1] = ndgrid(theta5, theta4, theta3, ...
        theta2, theta1);

    % Reshape the matrices THETAi_mat to arrays. Not the most efficient way of
    % doing it again!
    total_length = steps ^ groups;
    theta1_mat = reshape(THETA1, [total_length, 1]);
    theta2_mat = reshape(THETA2, [total_length, 1]);
    theta3_mat = reshape(THETA3, [total_length, 1]);
    theta4_mat = reshape(THETA4, [total_length, 1]);
    theta5_mat = reshape(THETA5, [total_length, 1]);


    %%%%%%%%%%%%%%%%%%%%%%% Optional %%%%%%%%%%%%%%%%%%%%%%
    % You can visualize a 3D space of your first 3 angles using the code below.
    % You should see a cubic grid!

    % scatter3(theta1_mat, theta2_mat, theta3_mat); grid on;
    % title('3D Grid of Joint Space', 'Interpreter','latex');
    % xlabel('$\theta_{1}\ (rad)$', 'Interpreter','latex'); 
    % ylabel('$\theta_{2}\ (rad)$', 'Interpreter','latex');
    % zlabel('$\theta_{3}\ (rad)$', 'Interpreter','latex');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    % Return matrix y. This contains all possible combinations of
    % [theta1, ..., theta5] inside their range.
    y = [theta1_mat, theta2_mat, theta3_mat, ...
        theta4_mat, theta5_mat];
end