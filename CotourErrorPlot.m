function []=CotourErrorPlot(TrueTipContourError,TrueOrienContourError,Proposed)

figure(6)
plot(Proposed(1,:),TrueTipContourError(:,4)*1e3,'b','Linewidth',1)
hold on
plot(Proposed(1,:),Proposed(3,:)*1e3,'r--','Linewidth',1);
xlabel('Time [sec]')
ylabel('Contour Error [micron]');
title('Comparision of True and Proposed Estimation Tool Tip Contour Errors');
legend('TrueTipContourError','EstimationTipContourErrors');
hold off;

figure(7)
plot(Proposed(1,:),(TrueTipContourError(:,4)'-Proposed(3,:))*1e3,'b','Linewidth',1)
hold on
% plot(Yang(1,:),(TrueTipContourError(:,4)'-Yang(2,:))*1e3,'r--','Linewidth',2);
xlabel('Time [sec]')
ylabel('Discrepency [micron]');
title('Discrepency of True and Estimation Tool Tip Contour Errors');
%legend('Proposed Method','Yang Method');
hold off;

figure(8)
plot(Proposed(1,:),TrueOrienContourError*1e3,'b','Linewidth',1)
hold on
plot(Proposed(1,:),Proposed(5,:)*1e3,'r--','Linewidth',1);
xlabel('Time [sec]')
ylabel('Contour Error [milliradians]');
title('Comparision of True and Proposed Estimation Tool Orientation Contour Errors');
legend('TrueOrienContourError','EstimationOrienContourErrors');
hold off;


figure(9)
plot(Proposed(1,:),(TrueOrienContourError'-Proposed(5,:))*1e6,'b','Linewidth',1)
%hold on
% plot(Yang(1,:),(TrueOrienContourError'-Yang(3,:))*1e6,'r--','Linewidth',2)
xlabel('Time [sec]')
ylabel('Discrepency [milliradians]');
title('Discrepency of True and Estimation Tool Orientation Contour Errors');
%legend('Proposed Method','Yang Method');
% hold off;