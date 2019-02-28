clear;
clc;
close all

figure (1)
ts = 0:0.001:5.122;
load('101.mat')
plot(ts,TrueTipContourError(:,end)*1e3,'b');
hold on
load('103.mat')
plot(ts,TrueTipContourError(:,end)*1e3,'r--');
xlabel('Time [s]');
ylabel('Tip contour Error [um]');
legend('Ro=0','Ro=0.001');
title('Comparsion of the tracking error and contour error');
axis([0 5.122 0 30]);

figure (2);
ts = 0:0.001:5.122;
load('102.mat')
plot(ts,TrueOrienContourError(:,end)*1e3,'b');
hold on
load('104.mat')
plot(ts,TrueOrienContourError(:,end)*1e3,'r--');
xlabel('Time [s]');
ylabel('Orient contour Error [urad]');
legend('Ro=0','Ro=0.001');
title('Comparsion of the tracking error and contour error');
axis([0 5.122 0 0.16])