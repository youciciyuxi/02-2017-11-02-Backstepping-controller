function [uc,IterativeTime,Difference,Flag] = Newton_Raphson(Positionspline,u0,Pc)
u = zeros(101,1);
u(1) = u0;

for i=1:200    
    P = deBoorRT(Positionspline, u(i), 0);
    Pdot = deBoorRT(Positionspline, u(i), 1);
    gdot = 2*Pdot'*(P'-Pc)';
    if  abs(gdot) < 1e-22
        uc = u(i);
        IterativeTime = i;
        Difference = (P'-Pc)*(P'-Pc)';
        Flag = 1;
        break;
    end
    u(i+1) = u(i) - (P'-Pc)*(P'-Pc)'/gdot;
    if u(i+1) < 0
        u(i+1) = 0;
    elseif u(i+1) > 1
       u(i+1) = 1;       
    end
    if (P'-Pc)*(P'-Pc)' < 1e-26
        uc = u(i);
        IterativeTime = i;
        Difference = (P'-Pc)*(P'-Pc)';
        Flag = 2;
        break;
    end
    uc = u(i);
    IterativeTime = i;
    Difference = (P'-Pc)*(P'-Pc)';
    Flag = 1;
end
