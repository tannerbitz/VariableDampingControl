clear all 
close all
clc
%------------------------------FORWARD KINEMATICS--------------------------
% addpath(genpath('rvctools'))
% 
% % DH PARAMETERS----------------------------------------------------------
% alpha = [-pi/2, pi/2, pi/2, -pi/2, -pi/2, pi/2, 0];
% a = [0, 0, 0, 0, 0, 0, 0];
% d = [0.360, 0, 0.420, 0, 0.400, 0, 0.126];
% theta = [0, 0, 0, 0, 0, 0, 0];
% dh = [theta' d' a' alpha'];
% 
% % BUILD ROBOT------------------------------------------------------------
% for i = 1:length(dh(:,1))
%     L{i} = Link('d', dh(i,2), 'a', dh(i,3), 'alpha', dh(i,4));
% end
% rob = SerialLink([L{1} L{2} L{3} L{4} L{5} L{6} L{7}]); 
% % teach(rob)
% % pause
% % TRAJECTORY-------------------------------------------------------------
% %move to start position
% q_zero = [0, 0, 0, 0, 0, 0, 0]*pi/180;
% [T_zero,all] = rob.fkine(q_zero);

%--------------------------------------------------------------------------

A=[350,400];
B=[500,400];
X=A(1)+B(1);
Y=A(2)+B(2);

v=A(1):B(1);
x1=0;
y1=.5;
x2=1;
y2=.5;
x3=.5;
y3=.5;
    R=.03;
    Ang = 0:0.01:2*pi; %angle from 0 to 2pi with increment of 0.01 rad. 
    CircX1=R*cos(Ang)+x1; 
    CircY1=R*sin(Ang)+y1;

    CircX2=R*cos(Ang)+x2; 
    CircY2=R*sin(Ang)+y2; 
    
x3=0;

    CircY3=R*sin(Ang)+y3; 
% figure('units','normalized','outerposition',[0 0 1 1])
figure('position',[200 200 500 500])
for i=1:1000000 %length(v)
    set(gcf,'Color','black')
    axis off
    hold on
%     axis off
if x3 <= 0
    inc = 0.01;  
elseif x3 >= 1
    inc = -0.01;       
else
%     return
end
    x3=x3+inc;

    CircX3=R*cos(Ang)+x3; 
    
    rectangle('Position',[0 0 1 1],'FaceColor','k');
    h1=plot(CircX1,CircY1,'LineWidth',5,'color','g');
    h2=plot(CircX2,CircY2,'LineWidth',5,'color','g');
    h2=plot(CircX3,CircY3,'LineWidth',5,'color','c');
%     h=area(Xpos+CircX,Ypos+CircY);
%     set(h,'FaceColor',[.7 0 0])
      axis off%colour of your circle in RGB, [0 0 0] black; [1 1 1] white 
%       set(h,'LineStyle','none')

%     rectangle('Position',[0 0 X Y],'FaceColor','k')
%     rectangle('Position',[A(1) A(2) 20 20],'FaceColor','b')
%     rectangle('Position',[B(1) B(2) 20 20],'FaceColor','b')

%     rectangle('Position',[0,0,X,Y],...
%   'Curvature',[0,0], 'FaceColor','b')
% axis square;
% %     
%     rectangle('Position',[1,1,1,1],...
%   'Curvature',[1,1], 'FaceColor','w')
% axis square;
% 
%     rectangle('Position',[B(1),B(2),X/10,Y/10],...
%   'Curvature',[1,1], 'FaceColor','w')
% axis square;
% 
%     rectangle('Position',[v(i),A(1),X/10,Y/10],...
%   'Curvature',[1,1], 'FaceColor','b')
% axis square;
% 
pause(0.001)
    clf
end
close all