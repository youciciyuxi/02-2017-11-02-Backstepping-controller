function []=ReferenceTrajPlot(ToolTipPosCoor,ToolOrienPosCoor,AxisDriveCommands,Curvature,Ts)

% Create figure1
figure1 = figure(1);
axes1 = axes('Parent',figure1);
view(axes1,[47.5 60]);
hold(axes1,'on');
plot3(ToolTipPosCoor(:,1),ToolTipPosCoor(:,2),ToolTipPosCoor(:,3),'b','Linewidth',2)
xlabel('X [mm]')
ylabel('Y [mm]')
zlabel('Z [mm]')
title('Tool tip position spline');
set(gca,'FontSize',14);
hold off;

% Create figure2
figure2 = figure(2);
[X,Y,Z] = sphere;
axes2 = axes('Parent',figure2,'CameraViewAngle',9.51951638462445,'DataAspectRatio',[1 1 1]);
view(axes2,[124.5 24]);
hold(axes2,'on');
surf(X,Y,Z,'Parent',axes2,'FaceLighting','none','EdgeLighting','flat','FaceColor','none','EdgeColor',[0.8 0.8 0.8]);
plot3(ToolOrienPosCoor(:,1),ToolOrienPosCoor(:,2),ToolOrienPosCoor(:,3),'LineWidth',2,'Color',[1 0 0]);
zlabel('Ok');
ylabel('Oj');
xlabel('Oi');
title('Tool orientation spline');
set(gca,'FontSize',14);
hold off

% Create figure3
figure(3)
t=Ts*(1:size(AxisDriveCommands,1));
plot(t,AxisDriveCommands(:,1),'r','Linewidth',2);
hold on;
plot(t,AxisDriveCommands(:,2),'--b','Linewidth',2);
hold on;
plot(t,AxisDriveCommands(:,3),'-.g','Linewidth',2);
hold off;
% title('Position - time of translational axes');
legend('X axis','Y axis','Z axis');
xlabel('Time [s]');
ylabel('Position [mm]');
set(gca,'FontSize',14);

% Create figure4
figure(4)
plot(t,AxisDriveCommands(:,4),'r','Linewidth',2);
hold on;
plot(t,AxisDriveCommands(:,5),'--b','Linewidth',2);
hold off;
% title('Position - time of rotary axes');
legend('A axis','C axis');
xlabel('Time [s]');
ylabel('Position [rad]');
set(get(gca,'XLabel'),'FontSize',14);%图上文字为8 point或小5号
set(get(gca,'YLabel'),'FontSize',14);


% Create figure5
figure(5)
plot(t,Curvature,'r','Linewidth',2);
xlabel('Time [s]');
ylabel('Curvature [1/mm]');
set(gca,'FontSize',14);

