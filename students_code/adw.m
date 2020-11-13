
function y = workspace_trajectory()

    steps = 20;         
    dimensions = 4; groups = dimensions;

    %joint angle ranges
    q1_min = -20*pi/180; q1_max = 20*pi/180;
    q2_min = -60*pi/180; q2_max = 60*pi/180;
    q3_min = -120*pi/180; q3_max = 0;
    q4_min = -90*pi/180; q4_max = 0;


    q1 = linspace(q1_min, q1_max, steps)';
    q2 = linspace(q2_min, q2_max, steps)';
    q3 = linspace(q3_min, q3_max, steps)';
    q4 = linspace(q4_min, q4_max, steps)';
    

    [Q4, Q3, Q2, Q1] = ndgrid(q4, q3, q2, q1);



    total_length = steps ^ groups;
    q1_mat = reshape(Q1, [total_length, 1]);
    q2_mat = reshape(Q2, [total_length, 1]);
    q3_mat = reshape(Q3, [total_length, 1]);
    q4_mat = reshape(Q4, [total_length, 1]);
    

    %%%%%%%%%%%%%%%%%%%%%% Optional %%%%%%%%%%%%%%%%%%%%%%
    % You can visualize a 3D space of your first 3 angles using the code below.
    % You should see a cubic grid!

    figure;
    scatter3(q1_mat, q2_mat, q3_mat); grid on;
    title('3D Grid of Joint Space', 'Interpreter','latex');
    xlabel('$\theta_{1}\ (rad)$', 'Interpreter','latex'); 
    ylabel('$\theta_{2}\ (rad)$', 'Interpreter','latex');
    zlabel('$\theta_{3}\ (rad)$', 'Interpreter','latex');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    y = [q1_mat, q2_mat, q3_mat, q4_mat];
end