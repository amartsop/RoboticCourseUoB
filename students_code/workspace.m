% attempted to use code from https://github.com/amartsop/RoboticCourseUoB/blob/master/WorkspaceTutorial.m

%calculation
traj = workspace_trajectory();
y = end_effector_position(traj); 
scatter3( y(:, 1), y(:, 2), y(:, 3), '*');
xlabel('x'); 
ylabel('y');  
zlabel('z');

%End effector  position function
function y = end_effector_position(q)
    %phalanx lengths
    l1 = 0.0398;
    l2 = 0.0224;
    l3 = 0.0158;
    % From the expressions of your final transformamtion matrix
    xe = l1*cos(q1)*cos(q2) + l3*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - l3*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + l2*cos(q1)*cos(q2)*cos(q3) - l2*cos(q1)*sin(q2)*sin(q3);
    ye = l1*cos(q2)*sin(q1) - l3*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - l3*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + l2*cos(q2)*cos(q3)*sin(q1) - l2*sin(q1)*sin(q2)*sin(q3);
    ze = - l1*sin(q2) - l3*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - l3*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - l2*cos(q2)*sin(q3) - l2*cos(q3)*sin(q2);
    y = [xe, ye, ze];
end

%workspace function
function y = workspace_trajectory()

    % "Steps" defines the number of discretizations. Is chosen the same for all 
    % thetai (i = 1, 2, 3, 4) The bigger the step the higher the accuracy.
    steps = 20;         

    % q is a 4d array for index
    dimensions = 4; groups = dimensions - 1;
    %joint angle ranges
    q1_min = -20*pi/180;
    q1_max = 20*pi/180;

    q2_min = -60*pi/180;
    q2_max = 60*pi/180;

    q3_min = -120*pi/180;
    q3_max = 0;

    q4_min = -90*pi/180;
    q4_max = 0;


    % Define the range of your joint angles thetai_min and thetai_max here.
    % These values depend on your problem. Try not to brake any thumbs!

    % Then caclulcate the linspace array based on the steps specified above.
    % Not the most efficient way of doing it but this is a pseudocode.
    q1 = linspace(q1_min, q1_max, steps)';
    q2 = linspace(q2_min, q2_max, steps)';
    q3 = linspace(q3_min, q3_max, steps)';
    q4 = linspace(q4_min, q4_max, steps)';
  
   
    % This is the essence of the code. It creates the n-dimensional grid 
    % (4 in this case)
    [Q4, Q3, Q2, Q1] = ndgrid(q4, q3, q2, q1);

    % Reshape the matrices THETAi_mat to arrays. Not the most efficient way of
    % doing it again!
    total_length = steps ^ groups;
    q1_mat = reshape(Q1, [total_length, 1]);
    q2_mat = reshape(Q2, [total_length, 1]);
    q3_mat = reshape(Q3, [total_length, 1]);
    q4_mat = reshape(Q4, [total_length, 1]);

    % Return matrix y. This contains all possible combinations of
    % [theta1, ..., theta5] inside their range.
    y = [q1_mat, q2_mat, q3_mat, q4_mat];
end


