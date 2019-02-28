function TipOrien = ForwardKinematics(DriveCommands)

Lacy = 0;
Lacz = 41;

HomePosDrive=[113.232633012931,18.4363460319427,0.239716132104988,0.686766850928449,-0.169983380596482];
% HomePosDrive=[0,0,0,0,0];

X = DriveCommands(1)+HomePosDrive(1);
Y = DriveCommands(2)+HomePosDrive(2);
Z = DriveCommands(3)+HomePosDrive(3);
A = DriveCommands(4)+HomePosDrive(4);
C = DriveCommands(5)+HomePosDrive(5);


Oi = sin(A)*sin(C);
Oj = sin(A)*cos(C);
Ok = cos(A);
Px = cos(C)*X+cos(A)*sin(C)*Y+sin(A)*sin(C)*Z-cos(A)*sin(C)*Lacy-sin(A)*sin(C)*Lacz;
Py = -sin(C)*X+cos(A)*cos(C)*Y+sin(A)*cos(C)*Z+(1-cos(A)*cos(C))*Lacy-sin(A)*cos(C)*Lacz;
Pz = -sin(A)*Y+cos(A)*Z+sin(A)*Lacy+(1-cos(A))*Lacz;

TipOrien = [Px Py Pz Oi Oj Ok]';
