function [Jp,Jo] = JacobinFunction(DriveCommands)

Lacy = 0;
Lacz = 41;

N = size(MotionCommands,1);

%HomePosDrive=[113.232633012931,18.4363460319427,0.239716132104988,0.686766850928449,-0.169983380596482];
HomePosDrive=[0,0,0,0,0];

X = DriveCommands(1)+HomePosDrive(1);
Y = DriveCommands(2)+HomePosDrive(2);
Z = DriveCommands(3)+HomePosDrive(3);
A = DriveCommands(4)+HomePosDrive(4);
C = DriveCommands(5)+HomePosDrive(5);

Jp = zeros(3,5,N);
Jo = zeros(3,2,N);

for i = 1:N
    % Tool orientation Jacobian Function
    Jp(:,:,i) = [cos(C(i))  cos(A(i))*sin(C(i))    sin(A(i))*sin(C(i))        -sin(A(i))*sin(C(i))*Y(i)+cos(A(i))*sin(C(i))*Z(i)+sin(A(i))*sin(C(i))*Lacy-cos(A(i))*sin(C(i))*Lacz     -sin(C(i))*X(i)+cos(A(i))*cos(C(i))*Y(i)+sin(A(i))*cos(C(i))*Z(i)-cos(A(i))*cos(C(i))*Lacy-sin(A(i))*cos(C(i))*Lacz
                -sin(C(i))  cos(A(i))*cos(C(i))    sin(A(i))*cos(C(i))        -sin(A(i))*cos(C(i))*Y(i)+cos(A(i))*cos(C(i))*Z(i)+sin(A(i))*cos(C(i))*Lacy-cos(A(i))*cos(C(i))*Lacz     -cos(C(i))*X(i)-cos(A(i))*sin(C(i))*Y(i)-sin(A(i))*sin(C(i))*Z(i)+cos(A(i))*sin(C(i))*Lacy+sin(A(i))*sin(C(i))*Lacz
                   0           -sin(A(i))              cos(A(i))                                -cos(A(i))*Y(i)-sin(A(i))*Z(i)+cos(A(i))*Lacy+sin(A(i))*Lacz                                                            0];

    % Tool orientation Jacobian Function
    Jo(:,:,i) = [cos(A(i))*sin(C(i))    sin(A(i))*cos(C(i))
                 cos(A(i))*cos(C(i))    -sin(A(i))*sin(C(i))
                 -sin(A(i))               0];
end