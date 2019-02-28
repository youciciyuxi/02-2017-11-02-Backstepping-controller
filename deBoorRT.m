function [C] = deBoorRT(Bspline, u, k)

%NURBS: BSpline Toolpath, including knots, control poitns, order, dimensions, and number of basis
%functions
%u: desired parameter
%k: order of the derviative
%#codegen
d = Bspline.dim;
p = Bspline.order-1;
n = Bspline.number-1;
U = Bspline.knots;
r = FindSpan(n,p,u,U)-1; %There's no 0 so it technically outputs span+1
Pd = zeros(d*(p+1),p+1);
P = zeros(d*(p+1),p+1);

P(1:d,:) = Bspline.coefs(:,(r-p)+1:r+1);

if(k>0)
    for b = 1:k
        m = b;
        for i = 0:p-m
            q = r-p+i;
            Pd(1:d,i+1) = ((p-m+1)/(U(q+p+2)-U(q+m+1)))*(P(1:d,i+2)-P(1:d,i+1));
        end
        P = Pd;
    end
end

p = p-k;
TempP = P(1:d,:);

for j = 1:p
    
   TempPPrev = TempP;
    
   for i = 0:(p-j)          
          a = (u-U(r-p+j+i+1))/(U(r-p+j+i+p-j+2)-U(r-p+j+i+1));
          TempP(:,i+1) = (1-a)*TempPPrev(:,i+1) + a*TempPPrev(:,i+2);  
    end
  
    P(j*d+1:j*d+d,:) = TempP;
    
end

C = P((p*d)+1:(p*d)+d,1);
end

%% FindSpan function called
% Written By: Alexander Yuen
% Last edited: 10/25/2011
% Function: Calculates degree, span index, size, and knot span index
% Inputs:
% n = span index
% p = degree
% u = desired knot
% U = knot vector

function [mid] = FindSpan(n,p,u,U)

if u == U(n+2) %check ends
    mid = n+1;
    'blaaaaaah';
    return
end

assert(u <= U(end), 'u is out of the range of U');
    

low = p+1; %start at first value
high = n+2; %m-p-1
mid = round((low+high)/2);

while (u < U(mid) || u >= U(mid+1))
    if(u < U(mid))
        high = mid;
    else
        low = mid;
    end
    mid = floor((low+high)/2);
end

end

