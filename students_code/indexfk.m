clear all
close all
clc

% %Transformation matrices
% syms q1 q2 q3 q4 l1 l2 l3
% T1 = [cos(q1) 0 -sin(q1) 0;sin(q1) 0 cos(q1) 0;0 -1 0 0;0 0 0 1];
% T2 = [cos(q2) -sin(q2) 0 l1*cos(q2);sin(q2) cos(q2) 0 l1*sin(q2);0 0 1 0;0 0 0 1];
% T3 = [cos(q3) -sin(q3) 0 l2*cos(q3);sin(q3) cos(q3) 0 l2*sin(q3);0 0 1 0;0 0 0 1];
% T4 = [cos(q4) -sin(q4) 0 l3*cos(q4);sin(q4) cos(q4) 0 l3*sin(q4);0 0 1 0;0 0 0 1];
% 
% TE = T1*T2*T3*T4;

%phalanx lengths
l1 = 0.0398;
l2 = 0.0224;
l3 = 0.0158;

%joint angle ranges
steps = 20;

q1_min = -20*pi/180;
q1_max = 20*pi/180;

q2_min = -60*pi/180;
q2_max = 60*pi/180;

q3_min = -120*pi/180;
q3_max = 0;

q4_min = -90*pi/180;
q4_max = 0;

q1 = linspace(q1_min, q1_max, steps)';
q2 = linspace(q2_min, q2_max, steps)';
q3 = linspace(q3_min, q3_max, steps)';
q4 = linspace(q4_min, q4_max, steps)';

%workspace using for loops

%!! coord_store = zeros(???) what size?
total_length = steps ^ groups + 1;
coord_store = zeros(total_length, 3);

count = 1;

for i = 1:length(q1)
    for j = 1:length(q2)
        for k = 1:length(q3)
            for m=1:length(q4)
                    count = count + 1;
                    xe = l1*cos(q1(i))*cos(q2(j)) + l3*cos(q4(m))*(cos(q1(i))*cos(q2(j))*cos(q3(k)) - cos(q1(i))*sin(q2(j))*sin(q3(k))) - l3*sin(q4(m))*(cos(q1(i))*cos(q2(j))*sin(q3(k)) + cos(q1(i))*cos(q3(k))*sin(q2(j))) + l2*cos(q1(i))*cos(q2(j))*cos(q3(k)) - l2*cos(q1(i))*sin(q2(j))*sin(q3(k));
                    ye = 0;
                    ze = 0;

                    % ye = l1*cos(q2(j))*sin(q1(i)) - l3*sin(q4(m))*(cos(q2(j))*sin(q1(i))*sin(q3(k)) + cos(q3(k))*sin(q1(i))*sin(q2(j))) - l3*cos(q4(m))*(sin(q1(i))*sin(q2(j))*sin(q3(k)) - cos(q2(j))*cos(q3(k))*sin(q1(i))) + l2*cos(q2(j))*cos(q3(k))*sin(q1(i)) - l2*sin(q1(i))*sin(q2(j))*sin(q3(k));
                    % ze = - l1*sin(q2(j)) - l3*cos(q4(m))*(cos(q2(j))*sin(q3(k)) + cos(q3(k))*sin(q2(j))) - l3*sin(q4(m))*(cos(q2(j))*cos(q3(k)) - sin(q2(j))*sin(q3(k)) - l2*cos(q2(j))*sin(q3(k)) - l2*cos(q3(k))*sin(q2(j));
                    y = [xe, ye, ze];
                    coord_store(count, :)=y;
                    count = count + 1;
            end
        end
    end
end
