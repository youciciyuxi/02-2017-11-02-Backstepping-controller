function  [TipContourErr,OrientationContourErr] = TipAndOrienConErrorCal(Trajectory,Pa,Pr,Oa)
%% Calculate the true tool tip contour error
Positionspline = Trajectory.positionspline;

M = size(Pa,2);
pp = fn2fm(Positionspline,'pp');
n = pp.pieces;
useg = pp.breaks(2:end)-pp.breaks(1:end-1);
du = 1e-3;
uin = du;
ktemp = 1;

hwait = waitbar(0,'Computing true tip contour errors,please wait...');
TipContourErr = zeros(M,4);
Pc = zeros(M,3);

for i = 1:M
    waitbar(i/M);
    
    while(1)
        start = fnval(pp,0);
        dstart = fnval(fnder(pp),0);
        if dot(Pa(:,i)-start,dstart) < 0 && ktemp == 1
            u = 0;
            break;
        end

        [u,fval,exitflag] = fzero(@(u)dot([pp.coefs(3*ktemp-2,1)*u^3+pp.coefs(3*ktemp-2,2)*u^2+pp.coefs(3*ktemp-2,3)*u^1+pp.coefs(3*ktemp-2,4)-Pa(1,i),...
                                           pp.coefs(3*ktemp-1,1)*u^3+pp.coefs(3*ktemp-1,2)*u^2+pp.coefs(3*ktemp-1,3)*u^1+pp.coefs(3*ktemp-1,4)-Pa(2,i),...
                                           pp.coefs(3*ktemp,1)*u^3+pp.coefs(3*ktemp,2)*u^2+pp.coefs(3*ktemp,3)*u^1+pp.coefs(3*ktemp,4)-Pa(3,i)],...
                                          [pp.coefs(3*ktemp-2,1)*3*u^2+pp.coefs(3*ktemp-2,2)*2*u^1+pp.coefs(3*ktemp-2,3),...
                                           pp.coefs(3*ktemp-1,1)*3*u^2+pp.coefs(3*ktemp-1,2)*2*u^1+pp.coefs(3*ktemp-1,3),...
                                           pp.coefs(3*ktemp,1)*3*u^2+pp.coefs(3*ktemp,2)*2*u^1+pp.coefs(3*ktemp,3)]), uin);

        if exitflag < 0 || u < -10^(-5) || u > useg(ktemp)    
            ktemp = ktemp+1;
            uin = du;
        else
            uin = max(u,du);   
            break;
        end
        
        if ktemp > n
            ktemp = n;
            u = useg(n);       
            break;
        end
    end
    
    Pc(i,1) = polyval(pp.coefs(3*ktemp-2,:),u);
    Pc(i,2) = polyval(pp.coefs(3*ktemp-1,:),u);
    Pc(i,3) = polyval(pp.coefs(3*ktemp,:),u);
    
    TipContourErr(i,1) = Pc(i,1)-Pa(1,i);
    TipContourErr(i,2) = Pc(i,2)-Pa(2,i);
    TipContourErr(i,3) = Pc(i,3)-Pa(3,i);
    TipContourErr(i,4) = norm(TipContourErr(i,1:3));
    
end      
close(hwait);


%% Calculate the responding parameter u
upa  = Trajectory.upa;
hwait = waitbar(0,'Computing true tip contour error parameters u ,please wait...');
uc = zeros(M,1);
u0 = zeros(M,1);
N = size(upa,2);
Iterative = zeros(M,1);
Difference = zeros(M,1);
Flag = zeros(M,1);
Kmax = 300;

for i = 1:M 
    waitbar(i/M);
    dtemp = zeros(Kmax,1);
    
    for k = 1:Kmax
        if i-Kmax/2+k < 1
            dtemp(k) = norm(Pr(1,:)-Pc(i,:));
        elseif i-Kmax/2+k > N
            dtemp(k) = norm(Pr(N,:)-Pc(i,:));
        else
            dtemp(k) = norm(Pr(i-Kmax/2+k,:)-Pc(i,:));
        end
    end
    [mind,index] = min(dtemp);
    if i-Kmax/2+index < 1
        u0(i) = upa(1);
    elseif i-Kmax/2+index > N
        u0(i) = upa(N);
    else
        u0(i) = upa(i-Kmax/2+index);
    end

    [uc(i),Iterative(i),Difference(i),Flag(i)] = Newton_Raphson(Positionspline,u0(i),Pc(i,:));
end
close (hwait);

%% Calculate the true tool orientation contour error
Orientationspline = Trajectory.orientationspline;

hwait = waitbar(0,'Computing true orientation contour errors,please wait...');
OrientationContourErr = zeros(M,1);

for i = 1:M 
    waitbar(i/M);
    
    Ou=deBoorRT(Orientationspline, uc(i), 0);
    Ou=Ou/norm(Ou);
    OrientationContourErr(i)=acos((Oa(i,:)*Ou)/(norm(Ou)*norm(Oa(i,:))));
end
close(hwait);
