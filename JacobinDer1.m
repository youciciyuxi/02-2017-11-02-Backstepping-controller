function JDer1 = JacobinDer1(PositionCommands,VelosityCommands)

Lacy = 0;
Lacz = 41;

HomePosDrive=[113.232633012931,18.4363460319427,0.239716132104988,0.686766850928449,-0.169983380596482];
%HomePosDrive=[0,0,0,0,0];

X = PositionCommands(1)+HomePosDrive(1);
Y = PositionCommands(2)+HomePosDrive(2);
Z = PositionCommands(3)+HomePosDrive(3);
A = PositionCommands(4)+HomePosDrive(4);
C = PositionCommands(5)+HomePosDrive(5);

XDer1 = VelosityCommands(1);
YDer1 = VelosityCommands(2);
ZDer1 = VelosityCommands(3);
ADer1 = VelosityCommands(4);
CDer1 = VelosityCommands(5);

JDer111 = -sin(C)*CDer1;
JDer112 = -sin(A)*sin(C)*ADer1+cos(A)*cos(C)*CDer1;
JDer113 = cos(A)*sin(C)*ADer1+sin(A)*cos(C)*CDer1;
JDer114=-sin(A)*sin(C)*YDer1+cos(A)*sin(C)*ZDer1-cos(A)*sin(C)*Y*ADer1-sin(A)*sin(C)*Z*ADer1+cos(A)*sin(C)*Lacy*ADer1+sin(A)*sin(C)*Lacz*ADer1-...
   sin(A)*cos(C)*Y*CDer1+cos(A)*cos(C)*Z*CDer1+sin(A)*cos(C)*Lacy*CDer1-cos(A)*cos(C)*Lacz*CDer1;
JDer115=-sin(C)*XDer1+cos(A)*cos(C)*YDer1+sin(A)*cos(C)*ZDer1-sin(A)*cos(C)*Y*ADer1+cos(A)*cos(C)*Z*ADer1+sin(A)*cos(C)*Lacy*ADer1-cos(A)*cos(C)*Lacz*ADer1-...
   cos(C)*X*CDer1-cos(A)*sin(C)*Y*CDer1-sin(A)*sin(C)*Z*CDer1+cos(A)*sin(C)*Lacy*CDer1+sin(A)*sin(C)*Lacz*CDer1;

JDer121 = -cos(C)*CDer1;
JDer122 = -sin(A)*cos(C)*ADer1-cos(A)*sin(C)*CDer1;
JDer123 = cos(A)*cos(C)*ADer1-sin(A)*sin(C)*CDer1;
JDer124=-sin(A)*cos(C)*YDer1+cos(A)*cos(C)*ZDer1-cos(A)*cos(C)*Y*ADer1-sin(A)*cos(C)*Z*ADer1+cos(A)*cos(C)*Lacy*ADer1+sin(A)*cos(C)*Lacz*ADer1+...
   sin(A)*sin(C)*Y*CDer1-cos(A)*sin(C)*Z*CDer1-sin(A)*sin(C)*Lacy*CDer1+cos(A)*sin(C)*Lacz*CDer1;
JDer125=-cos(C)*XDer1-cos(A)*sin(C)*YDer1-sin(A)*sin(C)*ZDer1+sin(A)*sin(C)*Y*ADer1-cos(A)*sin(C)*Z*ADer1-sin(A)*sin(C)*Lacy*ADer1+cos(A)*sin(C)*Lacz*ADer1+...
    sin(C)*X*CDer1-cos(A)*cos(C)*Y*CDer1-sin(A)*cos(C)*Z*CDer1+cos(A)*cos(C)*Lacy*CDer1+sin(A)*cos(C)*Lacz*CDer1;

JDer131 = 0;
JDer132 = -cos(A)*ADer1;
JDer133 = -sin(A)*ADer1;
JDer134=-cos(A)*YDer1-sin(A)*ZDer1+sin(A)*Y*ADer1-cos(A)*Z*ADer1-sin(A)*Lacy*ADer1+cos(A)*Lacz*ADer1;
JDer135=0;

JDer144 = -sin(A)*sin(C)*ADer1+cos(A)*cos(C)*CDer1;
JDer145 = cos(A)*cos(C)*ADer1-sin(A)*sin(C)*CDer1;

JDer154 = -sin(A)*cos(C)*ADer1-cos(A)*sin(C)*CDer1;
JDer155 = -cos(A)*sin(C)*ADer1-sin(A)*cos(C)*CDer1;

JDer164 = -cos(A)*ADer1;
JDer165 = 0;


JDer1 = [ JDer111  JDer112    JDer113    JDer114     JDer115
          JDer121  JDer122    JDer123    JDer124     JDer125
          JDer131  JDer132    JDer133    JDer134     JDer135
            0          0            0    JDer144     JDer145    
            0          0            0    JDer154     JDer155
            0          0            0    JDer164     JDer165];
end

