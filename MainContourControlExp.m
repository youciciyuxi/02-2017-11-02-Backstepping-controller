clc
clear
close all
%% Set general condition 
Ts=0.001;                                                           % Sample Period: 1ms 
VlimitX = 6; VlimitY = 6; VlimitZ = 6; VlimitA = 5; VlimitC = 5;    % Drives' input saturation value

%% Load trajectory data and Motion commands for single drives 
InterpolationData=load('InterpInfo_Zhao_Fan.mat');
% InterpolationData=load('InterpInfo_Zhao_Blade.mat');
ToolTipPos=InterpolationData.interpcor(:,1:3);

X=ToolTipPos(:,1);                                                      % Set the parameters for C-MEX S function for hunting the foot point
Y=ToolTipPos(:,2);
Z=ToolTipPos(:,3);
U=InterpolationData.interpcor(:,9);
V=InterpolationData.interpcor(:,7);
A=InterpolationData.interpcor(:,8);

%ToolOrienPos=InterpolationData.interpcor(:,4:6);
% Indentified the orientation vectors
lenOrienVector=length(U);
ToolOrienPos=zeros(lenOrienVector,3);
for ii=1:lenOrienVector
    TempOrienVector=InterpolationData.interpcor(ii,4:6);
    ToolOrienPos(ii,1:3)=InterpolationData.interpcor(ii,4:6)/norm(TempOrienVector);
end

I=ToolOrienPos(:,1);
J=ToolOrienPos(:,2);
K=ToolOrienPos(:,3);

DriveCommands =InverseKinematics_DH(ToolTipPos,ToolOrienPos);
HomePosDrive=DriveCommands(1,:);                                        % Set the home positon for motion control
Xcommands = DriveCommands(:,1)-HomePosDrive(1);            
Ycommands = DriveCommands(:,2)-HomePosDrive(2);          
Zcommands = DriveCommands(:,3)-HomePosDrive(3);  
Acommands = DriveCommands(:,4)-HomePosDrive(4);  
Ccommands = DriveCommands(:,5)-HomePosDrive(5);                         % To make sure the trajectory is from 0 position


len=length(U);
DeBoorPointDer012=zeros(3,3,len);
for jj=1:len
    DeBoorPointDer012(:,:,jj)=DeBoorCoxNurbsCal(U(jj));
end
Curvature = CurvatureCal(DeBoorPointDer012);

%ReferenceTrajPlot(ToolTipPos,ToolOrienPos,DriveCommands,Curvature,Ts);

ProfileVelosity=InterpolationData.interpcor(:,7);                                % Set the additional information for Five-axis controller
ProfileAcceleration=InterpolationData.interpcor(:,8);
uNurbsPara=InterpolationData.interpcor(:,9);

len2=length(uNurbsPara);
DataPointDer1=zeros(len2,3);
DataPointDer2=zeros(len2,3);
ProfileVelX=zeros(len2,1);
ProfileVelY=zeros(len2,1);
ProfileVelZ=zeros(len2,1);
ProfileAccX=zeros(len2,1);
ProfileAccY=zeros(len2,1);
ProfileAccZ=zeros(len,1);

for ii=1:len2
    [DeBoorP]=DeBoorCoxNurbsCal(uNurbsPara(ii));
    DataPointDer1(ii,:)=DeBoorP(2,:);
    DataPointDer2(ii,:)=DeBoorP(3,:);
    xKnotCorDer1=DataPointDer1(ii,1);
    yKnotCorDer1=DataPointDer1(ii,2);
    zKnotCorDer1=DataPointDer1(ii,3);
    xKnotCorDer2=DataPointDer2(ii,1);
    yKnotCorDer2=DataPointDer2(ii,2);
    zKnotCorDer2=DataPointDer2(ii,3);
    % tangentVector Calculation
    tangentVectorX=xKnotCorDer1/sqrt(xKnotCorDer1^2+yKnotCorDer1^2+zKnotCorDer1^2);
    tangentVectorY=yKnotCorDer1/sqrt(xKnotCorDer1^2+yKnotCorDer1^2+zKnotCorDer1^2);
    tangentVectorZ=zKnotCorDer1/sqrt(xKnotCorDer1^2+yKnotCorDer1^2+zKnotCorDer1^2);
    % subNormalVector1 Calculation
    subNormalVectorX1=yKnotCorDer1*zKnotCorDer2-yKnotCorDer2*zKnotCorDer1;
    subNormalVectorY1=-xKnotCorDer1*zKnotCorDer2+xKnotCorDer2*zKnotCorDer1;
    subNormalVectorZ1=xKnotCorDer1*yKnotCorDer2-xKnotCorDer2*yKnotCorDer1;
    % subNormalVector Calculation
    subNormalVectorX=subNormalVectorX1/sqrt(subNormalVectorX1^2+subNormalVectorY1^2+subNormalVectorZ1^2);
    subNormalVectorY=subNormalVectorY1/sqrt(subNormalVectorX1^2+subNormalVectorY1^2+subNormalVectorZ1^2);
    subNormalVectorZ=subNormalVectorZ1/sqrt(subNormalVectorX1^2+subNormalVectorY1^2+subNormalVectorZ1^2);
    % normalVector Calculation
    normalVectorX=subNormalVectorY*tangentVectorZ-subNormalVectorZ*tangentVectorY;
    normalVectorY=subNormalVectorZ*tangentVectorX-subNormalVectorX*tangentVectorZ;
    normalVectorZ=subNormalVectorX*tangentVectorY-subNormalVectorY*tangentVectorX;
    % transformation matrix notation
    t=[tangentVectorX,tangentVectorY,tangentVectorZ]';
    n=[normalVectorX,normalVectorY,normalVectorZ]';
    % Curvature Calculation
    KnotCorDer1=[xKnotCorDer1,yKnotCorDer1,zKnotCorDer1];
    KnotCorDer2=[xKnotCorDer2,yKnotCorDer2,zKnotCorDer2];
    subNormalVector=[subNormalVectorX,subNormalVectorY,subNormalVectorZ];
    sigma=norm(KnotCorDer1);
    Curvature2=cross(KnotCorDer1,KnotCorDer2)*subNormalVector'/sigma^3;
    ProfileVelX(ii)=ProfileVelosity(ii)*t(1);
    ProfileVelY(ii)=ProfileVelosity(ii)*t(2);
    ProfileVelZ(ii)=ProfileVelosity(ii)*t(3);
    ProfileAccX(ii)=ProfileVelosity(ii)^2*Curvature2*n(1)+ProfileAcceleration(ii)*t(1);
    ProfileAccY(ii)=ProfileVelosity(ii)^2*Curvature2*n(2)+ProfileAcceleration(ii)*t(2);
    ProfileAccZ(ii)=ProfileVelosity(ii)^2*Curvature2*n(3)+ProfileAcceleration(ii)*t(3);
end

%% Set the plant X,Y,Z,A,Z parameters
[PlantXPara,PlantYPara,PlantZPara,PlantAPara,PlantCPara]=PlantParameters(Ts);
PlantXNum=[PlantXPara.b1 PlantXPara.b0];
PlantXDen=[1 PlantXPara.a1 PlantXPara.a0];
PlantYNum=[PlantYPara.b1 PlantYPara.b0];
PlantYDen=[1 PlantYPara.a1 PlantYPara.a0];
PlantZNum=[PlantZPara.b1 PlantZPara.b0];
PlantZDen=[1 PlantZPara.a1 PlantZPara.a0];
PlantANum=[PlantAPara.b1 PlantAPara.b0];
PlantADen=[1 PlantAPara.a1 PlantAPara.a0];
PlantCNum=[PlantCPara.b1 PlantCPara.b0];
PlantCDen=[1 PlantCPara.a1 PlantCPara.a0];

c1_x = 2;
c1_y = c1_x;
c1_z = c1_x;
k_x = 1;
k_y = k_x;
k_z = k_x;

c1_i = 0.2;
c1_j = c1_i;
c1_k = c1_i;
k_i = 0.2;
k_j = k_i;
k_k = k_i;

R1 = 0.8;
R2 = R1;
R3 = R1;
R4 = 0.8;
R5 = R4;
R6 = R4;


%% Simulate the model
% sim('BSCForFiveAxisExperiment01');
% 
% %% Calculate true tool tip contour error and tool orientation contour error
% ActualPosition=load('ActualPosition.mat');
% Trajectory = load('Trajectory_Zhao_Fan.mat');
% ProposedMethod=load('ConErrorProposed.mat');
% %YangMethod=load('ConErrorYang.mat');
% 
% [M,N]=size(ActualPosition.ActualPos);
% ActualPositionData=zeros(M-1,N);
% for ss=1:N
%     ActualPositionData(1,ss)=ActualPosition.ActualPos(2,ss)+HomePosDrive(1);
%     ActualPositionData(2,ss)=ActualPosition.ActualPos(3,ss)+HomePosDrive(2);
%     ActualPositionData(3,ss)=ActualPosition.ActualPos(4,ss)+HomePosDrive(3);
%     ActualPositionData(4,ss)=ActualPosition.ActualPos(5,ss)+HomePosDrive(4);
%     ActualPositionData(5,ss)=ActualPosition.ActualPos(6,ss)+HomePosDrive(5);
% end
% [Pa,Oa] = ForwardKinematics_DH(ActualPositionData');
% Pr=ToolTipPos;
% [TrueTipContourError,TrueOrienContourError]=TipAndOrienConErrorCal(Trajectory,Pa',Pr,Oa);

%% Plot the estimated contour and the true contour error
% CotourErrorPlot(TrueTipContourError,TrueOrienContourError,ProposedMethod.Proposed);



