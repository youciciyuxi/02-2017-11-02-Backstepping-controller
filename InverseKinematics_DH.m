function [DriveCommand] = InverseKinematics_DH(Pw_Ref,Ow_Ref)

Lacy = 0;
Lacz = 41;

Px = Pw_Ref(:,1);
Py = Pw_Ref(:,2);
Pz = Pw_Ref(:,3);
Oi = Ow_Ref(:,1);
Oj = Ow_Ref(:,2);
Ok = Ow_Ref(:,3);

N = size(Pw_Ref,1);
X = zeros(N,1);
Y = zeros(N,1);
Z = zeros(N,1);
A = zeros(N,1);
C = zeros(N,1);


for i = 1:N
    A(i) = acos(Ok(i));
    if i==1
        C(i) = atan2(Oi(i),Oj(i));
    else
        C(i) = atan2(Oi(i),Oj(i));
        if abs(C(i)-C(i-1))>=6.2
            C(i)=C(i)-2*pi;
        end
    end
    X(i) = cos(C(i))*Px(i)-sin(C(i))*Py(i)+sin(C(i))*Lacy;
    Y(i) = cos(A(i))*sin(C(i))*Px(i)+cos(A(i))*cos(C(i))*Py(i)-sin(A(i))*Pz(i)+(1-cos(A(i))*cos(C(i)))*Lacy+sin(A(i))*Lacz;
    Z(i) = sin(A(i))*sin(C(i))*Px(i)+sin(A(i))*cos(C(i))*Py(i)+cos(A(i))*Pz(i)-sin(A(i))*cos(C(i))*Lacy+(1-cos(A(i)))*Lacz;
end

DriveCommand = [X,Y,Z,A,C];

