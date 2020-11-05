%% INDEX FINGER Group_11
% Modelling the index finger with 4DOF

%% A series of joint angles
% The following variables are defined in the form of column-vectors with
% 4 rows each. Each row represents a different position (angle) of the joint.
q1 = [-45 -40 -35 -30]'*pi/180 ;
q2 = [30 35 40 45]'*pi/180 ;
q3 = [30 35 40 45]'*pi/180 ;
q4 = [30 35 40 45]'*pi/180 ;
q = [ q1 q2 q3 q4] ;


%% Links Lengths
l1 = 1 ;
l2 = 0.5 ;
l3 = 0.2 ;

%% Trigonometric abbreviations
c1 = cos(q1);
c2 = cos(q2);
c3 = cos(q3);
c4 = cos(q4);
c12 = cos(q1+q2);
c123 = cos(q1+q2+q3);

s1= sin(q1);
s2 = sin(q2);
s3 = sin(q3);
s4 = sin(q4);
s12 = sin(q1+q2);
s123 = sin(q1+q2+q3);

%% Tip position
% These equations are derived from the Forward Kinematic model of the 4DOF robot
xt = l1*c1+l3*c123+l2*c12 ;
%yt = 0;
zt = -l1*s1-l3*s123-l2*s12 ;

%pt = [ xt yt zt ] ;
pt = [ xt zt ] ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the trajectory of the end-effector
% figure (1)
% % set(1,'position',[680 558 560 420])
% 
% plot3(pt(1,1),pt(1,2),0,'rx')       % plot the first position of the robot's end effector
% hold on
% plot(pt(2:4,1),pt(2:4,2),'x')       % plot the 3 following positions of the robot's end effector
% title('Tip Trajectory') ; xlabel('x (m)') ; ylabel('y (m)') ;


%% Plot the robotic arm, in 4 different positions
figure (2) 
% set(2,'position',[116 190 560 420])

x1 = zeros(4,1) ;
%y1 = zeros(4,1) ;
z1 = zeros(4,1) ;

x2 = l1*c1 ;
%y2 = zeros(4,1) ;
z2 = -l1*s1 ;

x3 = l2*c12+l1*c1 ;
%y3 = zeros(4,1) ;
z3 = -s12*l2-l1*s1;

for i = 1:4
    xx = [ x1(i); x2(i); x3(i); pt(i,1) ] ;
    %yy = [ y1(i); y2(i); y3(i); pt(i,2) ] ;
    zz = [ z1(i); z2(i); z3(i); pt(i,2) ] ;
    
    %plot(xx,yy,zz,'ko-','Linewidth',2)
    plot(xx,zz,'ko-','Linewidth',2)
    axis equal
    hold on
        
    %xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)') ;
    %text(pt(1,1),pt(1,2),pt(1,3),'x') ; text(pt(1,1) + 0.002,pt(1,2) + 0.002,pt(1,3) + 0.002,'ptStart') ;
    %text(pt(4,1),pt(4,2),pt(4,3),'x') ; text(pt(4,1) + 0.002,pt(4,2) + 0.002,pt(4,3) + 0.002,'ptEnd') ;
    xlabel('x (m)') ; zlabel('z (m)') ;
    text(pt(1,1),pt(1,2),'x') ; text(pt(1,1) + 0.002,pt(1,2) + 0.002,'ptStart') ;
    text(pt(4,1),pt(4,2),'x') ; text(pt(4,1) + 0.002,pt(4,2) + 0.002,'ptEnd') ;
    
    axis([-1.8 1.8 -1.8 1.8])
    pause(0.05)
    hold off
    pause(1)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Workspace
% q1 = 0:1:180 ; %pi/2 ; 
% q2 = -90:1:180 ; %pi/2 ; 
% 
% cw1 = cosd(q1);
% sw1 = sind(q1);
% cw12 = zeros(length(q1),length(q2)) ;
% sw12 = zeros(length(q1),length(q2));
% 
% %% Plot the workspace of the robot
% figure (3)
% % set(3,'position',[1243 190 560 420])
% 
% xwork = zeros(length(q1),length(q2)) ; % reserving space for the variables, because
% ywork = zeros(length(q1),length(q2)) ; % otherwise they would be created later within a loop.
% 
% for i = 1:length(q1)	% for q1
%     for j = 1:length(q2)   % for q2
%         cw12(i,j) = cosd(q1(1,i)+ q2(1,j));
%         sw12(i,j) = sind(q1(1,i)+ q2(1,j));
%         xwork(i,j) = l1*cw1(i)+l2*cw12(i,j) ;
%         ywork(i,j) = l1*sw1(i)+l2*sw12(i,j) ;
%     end
% end
% 
% axis([-0.5 1.5 0 1.5])
% plot(xwork,ywork,'.')
% hold on
% grid on
% 
% % circle(0,0,l1+l2)
% % plot(0,0,'r*')
% % circle(l1,0,l2)
% % plot(l1,0,'r*')
% % circle(0,l1,l2)
% % plot(0,l1,'r*')
% % circle(0,0,sqrt(l1^2+l2^2))
% % % circle(0,0,l1-l2)
% axis equal
% xlabel('x (m)')
% ylabel('y (m)')