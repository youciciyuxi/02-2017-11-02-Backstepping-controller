function J = Jacobin(DriveCommands)

Lacy = 0;
Lacz = 41;

HomePosDrive=[113.232633012931,18.4363460319427,0.239716132104988,0.686766850928449,-0.169983380596482];
%HomePosDrive=[0,0,0,0,0];

X = DriveCommands(1)+HomePosDrive(1);
Y = DriveCommands(2)+HomePosDrive(2);
Z = DriveCommands(3)+HomePosDrive(3);
A = DriveCommands(4)+HomePosDrive(4);
C = DriveCommands(5)+HomePosDrive(5);

 
J = [   cos(C)  cos(A)*sin(C)    sin(A)*sin(C)        -sin(A)*sin(C)*Y+cos(A)*sin(C)*Z+sin(A)*sin(C)*Lacy-cos(A)*sin(C)*Lacz     -sin(C)*X+cos(A)*cos(C)*Y+sin(A)*cos(C)*Z-cos(A)*cos(C)*Lacy-sin(A)*cos(C)*Lacz
       -sin(C)   cos(A)*cos(C)    sin(A)*cos(C)       -sin(A)*cos(C)*Y+cos(A)*cos(C)*Z+sin(A)*cos(C)*Lacy-cos(A)*cos(C)*Lacz     -cos(C)*X-cos(A)*sin(C)*Y-sin(A)*sin(C)*Z+cos(A)*sin(C)*Lacy+sin(A)*sin(C)*Lacz
       0          -sin(A)            cos(A)                 -cos(A)*Y-sin(A)*Z+cos(A)*Lacy+sin(A)*Lacz                                           0
       0            0                  0                                  cos(A)*sin(C)                                                     sin(A)*cos(C)
       0            0                  0                                  cos(A)*cos(C)                                                     -sin(A)*sin(C)
       0            0                  0                                     -sin(A)                                                             0  ];


end



