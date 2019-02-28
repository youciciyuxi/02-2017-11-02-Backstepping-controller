function [Pw,Ow] = ForwardKinematics_DH(DriveCommands)

Lacy = 0;
Lacz = 41;

X = DriveCommands(:,1);
Y = DriveCommands(:,2);
Z = DriveCommands(:,3);
A = DriveCommands(:,4);
C = DriveCommands(:,5);

N = size(DriveCommands,1);
Oi = zeros(N,1);
Oj = zeros(N,1);
Ok = zeros(N,1);
Px = zeros(N,1);
Py = zeros(N,1);
Pz = zeros(N,1);

for i = 1:N
    Oi(i) = sin(A(i))*sin(C(i));
    Oj(i) = sin(A(i))*cos(C(i));
    Ok(i) = cos(A(i));
    Px(i) = cos(C(i))*X(i)+cos(A(i))*sin(C(i))*Y(i)+sin(A(i))*sin(C(i))*Z(i)-cos(A(i))*sin(C(i))*Lacy-sin(A(i))*sin(C(i))*Lacz;
    Py(i) = -sin(C(i))*X(i)+cos(A(i))*cos(C(i))*Y(i)+sin(A(i))*cos(C(i))*Z(i)+(1-cos(A(i))*cos(C(i)))*Lacy-sin(A(i))*cos(C(i))*Lacz;
    Pz(i) = -sin(A(i))*Y(i)+cos(A(i))*Z(i)+sin(A(i))*Lacy+(1-cos(A(i)))*Lacz;
end

Pw = [Px,Py,Pz];
Ow = [Oi,Oj,Ok];


