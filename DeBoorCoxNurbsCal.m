function [DeBoorP] = DeBoorCoxNurbsCal(u)

% 几个要用到的全局变量，使用全局变量减少数据传递次数
global KnotVector;  % 节点向量
global CtrPoints;          % 控制点
global curveDegree; % 阶数
global CtrolPointMulweight; % 坐标与权值的乘积

%% 初始化两条五轴刀路的曲线参数，Fan 曲线和Blade 曲线
KnotVector=[0,0,0,0,0.106505793441501,0.157284527891952,0.199426140710703,0.234151035701386,0.266827885254851,0.300976958184472,...
    0.340545525832759,0.387257035050466,0.433031806140371,0.471312812708211,0.498755113378692,0.523379024673771,0.546560267841075,0.571456154181000,...
    0.601575236029666,0.644990504057629,0.695759844487100,0.749760773458549,0.799359778440787,0.849103852157646,0.899353829759719,1,1,1,1];
curveDegree=3;
xCtrlCoor=[113.560775000000,117.907784172815,121.462357619208,106.012714741771,96.1401560617727,89.1911507101181,79.7798213582500,...
    71.6724064108289,64.9824828874336,54.8424637547536,39.2392540175040,32.8933055388342,26.4490644896035,22.4116145178198,18.2736073139743,16.6339019865314,...
    16.3334292670881,21.2856317933606,27.4817364686037,29.4722901725788,23.6369028383090,6.32107092312859,-18.9344035716273,-38.6283247647271,...
    -49.4388780000000];
yCtrlCoor=[7.73526600000000,-3.72550226174048,-27.3277457365089,-54.8011679321253,-64.2516317460606,-66.4558822681257,...
    -65.6471075625930,-61.0876209490450,-53.9448519849804,-39.2705965087025,-23.8067759915909,-18.8923033586951,-16.6641134539066,-16.1633900205040,...
    -17.6184300123742,-21.6391152140663,-28.6880626467441,-39.9762303990255,-68.3782350657572,-85.5601869775015,-105.212470043406,-115.316836001493,...
    -120.008716617807,-115.952845290824,-108.784390000000];
zCtrlCoor=[-2.20931400000000,-1.40854242697360,-0.0607452135072286,2.65001386114913,3.34041259418642,6.37847216653963,...
    7.21905148949902,6.92801120194676,5.95234207426767,4.96872130112414,3.57701522569175,3.19929368574463,2.41772678760589,2.22552494327216,...
    0.164416924317616,-1.71809989263841,-2.92674119711899,-3.69925250451544,-5.59355790378744,-6.82029075680885,-4.51969660571512,-1.66864654212492,...
    -0.789248427751609,1.11844292823757,2.08953700000000];

% KnotVector=[0,0,0,0,0.0408270951965416,0.0611940355785803,0.0815127102812400,0.0998068519177771,0.116168475013227,0.132453051992392,0.150681786178957,...
%     0.170642433374905,0.190571907488079,0.210343983298041,0.230062487410986,0.249627968419188,0.269109785813314,0.288533011464667,0.307925663610566,0.327310086207321,...
%     0.346889505719849,0.366471914550031,0.386108414230578,0.405539455343359,0.424931658074850,0.444196974894897,0.466147053560963,0.490188087009663,0.514656518219301,...
%     0.536800316606819,0.557236016663954,0.577690222259982,0.596120480446577,0.612792136648440,0.629402593414203,0.648008564290944,0.668344678661991,0.688679818231071,...
%     0.708993354905670,0.729278370921988,0.749533326892922,0.769747806784225,0.789916342472583,0.810036746374911,0.830119399259965,0.850162193357179,0.870171676526078,...
%     0.890158495621789,0.910154164943106,0.928451988082027,0.944847257783646,0.961315507500795,1,1,1,1];
% curveDegree=3;
% xCtrlCoor=[74.9684000000000,72.7433531113297,69.4637463440642,65.0564602973189,62.0648681491990,58.9474230399046,57.3738067915712,55.7673603964895,53.0803170932839,...
%     50.7515655695079,48.5505919169795,46.5125171958761,44.7297238106929,43.1179294396447,41.6580862419665,40.3504440354397,38.8790602860284,37.3158823163231,...
%     35.6127949206256,33.6495185413456,31.6763321538687,29.5259515641397,27.2688029982275,24.9715958788479,21.5845853639681,22.0068232495600,27.5110221341699,...
%     29.7846400053149,32.3695636363164,34.4157367449989,36.5457493551781,37.5615231483619,38.6810504405495,40.4307826897082,41.9434919544858,43.4477562029341,...
%     44.7722668475837,46.1722217939660,47.7101667299969,49.4107495030916,51.2874592271498,53.3695196600161,55.6645270953027,58.1734642262872,60.8758797313934,...
%     63.7863172857487,66.6480873973455,69.8035615330687,71.6947188416029,74.1158188738777,78.1522289421750,79.8596999999999];
% yCtrlCoor=[-35.1178000000000,-34.8449904292686,-34.3985591498241,-33.5690570175588,-33.0342605321579,-32.5949922401735,-32.4319262520343,-32.2715461751259,...
%     -32.1634418342067,-32.2698950810537,-32.5775550234464,-33.1226257540289,-33.8839286347776,-34.8703154043207,-36.0495171416578,-37.4346995857198,-38.8733536038383,...
%     -40.2664486975342,-41.6824247242068,-43.1790051529728,-44.5375592011837,-45.8770977141321,-47.1212683303803,-48.2749153056296,-49.6424849960614,-49.6136149680830,...
%     -46.5831227724961,-45.0382700545476,-43.2275434747532,-41.6564737096269,-39.8686957673871,-38.9430605455226,-37.9015078068679,-36.1713236050619,-34.5620136751747,...
%     -32.8858549591366,-31.2413829542469,-29.7703772135650,-28.5319198684234,-27.5230010502209,-26.7494978662432,-26.2090240771741,-25.8998857343956,-25.8134276114032,...
%     -25.9169450066287,-26.2005192296588,-26.6114501535619,-27.1524155150160,-27.4216594623506,-27.7416473129740,-28.1852301220492,-28.3110000000000];
% zCtrlCoor=[112.894600000000,113.279790703642,113.913163717178,114.699161423338,115.515458370962,116.850849212101,117.703160809429,118.657227373733,120.565262603178,...
%     122.634588917956,124.915787414865,127.388016145654,129.861359852539,132.353960697068,134.815043297329,137.227902527975,139.493660685232,141.759416735934,143.842064287883,...
%     145.973984984107,147.838107352694,149.690958878137,151.284384493880,153.241065205788,153.493021454625,162.602594082611,158.803277097218,157.532151167653,155.595250167404,...
%     153.843128542818,151.672240823165,150.535479820709,149.172632353644,146.872410336341,144.575069417132,142.034141386636,139.425683915361,136.753809997969,134.053979007558,...
%     131.367055839086,128.745063406684,126.236803006082,123.898728339281,121.781030330285,119.959264781041,118.454864236079,117.393673060959,116.668368613364,116.215488535698,...
%     115.665622503005,114.803170916957,114.507500000000];

CtrPoints=[xCtrlCoor' yCtrlCoor' zCtrlCoor'];
weightVector = ones(1,length(xCtrlCoor))';

n = size(CtrPoints, 1);

uIndex = findSpan(n, curveDegree, u, KnotVector);

CtrolPointMulweight=[CtrPoints(:,1).*weightVector,CtrPoints(:,2).*weightVector,CtrPoints(:,3).*weightVector];

% 系数矩阵
alfaMatrix = zeros(curveDegree, curveDegree);
alfaMatrixDer1 = zeros(curveDegree, curveDegree);
alfaMatrixDer2 = zeros(curveDegree, curveDegree);

% 计算型值点的系数矩阵
for tempData = 0 : curveDegree - 1
    for tempData2 = 0 : tempData
        utemp =  (KnotVector(uIndex + tempData2 + 2) - KnotVector(uIndex - tempData + tempData2 + 1));
        
        if utemp == 0
            alfaMatrix(tempData2 + 1, tempData + 1) = 0;
        else
            alfaMatrix(tempData2 + 1, tempData + 1) = (u - KnotVector(uIndex - tempData + tempData2 + 1)) / utemp;
        end
    end
end

% 计算型值点
deBoorAu = DeBoorCoxCal(alfaMatrix, CtrolPointMulweight, uIndex);            
deBoorBu = DeBoorCoxCal(alfaMatrix, weightVector, uIndex);

if deBoorBu == 0
    KnotCoor = [0 0 0];
else
    KnotCoor = deBoorAu / deBoorBu;
end
DeBoorP(1, :) = KnotCoor;

% 计算一阶导矢需要用到的系数矩阵
for tempData = curveDegree - 1 : -1 : 1
    for tempData2 = tempData: - 1 : 1
        utemp = (KnotVector(uIndex + tempData2 + 1) - KnotVector(uIndex - tempData + tempData2 + 1));
        if utemp == 0
            alfaMatrixDer1(tempData2 + 1, tempData + 1) = 0;
        else
            alfaMatrixDer1(tempData2 + 1, tempData + 1) = (u - KnotVector(uIndex - tempData + tempData2 + 1)) / utemp;
        end
    end
end

% 计算二阶导矢需要用到的系数矩阵
for tempData = curveDegree - 1: -1 : 2
    for tempData2 = tempData : -1 : 2
        utemp = (KnotVector(uIndex + tempData2) - KnotVector(uIndex - tempData + tempData2 + 1));
        if utemp == 0
            alfaMatrixDer2(tempData + 1, tempData + 1) = 0;
        else
            alfaMatrixDer2(tempData + 1, tempData + 1) = (u - KnotVector(uIndex - tempData + tempData2 + 1)) / utemp;
        end
    end
end

deBoorDer1 = DeBoorCoxDer1Cal(alfaMatrixDer1, CtrolPointMulweight, uIndex);
deBoorDer1Bu = DeBoorCoxDer1Cal(alfaMatrixDer1, weightVector, uIndex);

if deBoorBu == 0
    KnotCoorDer1 = [0 0 0];
else
    KnotCoorDer1 = (deBoorDer1 - deBoorDer1Bu * KnotCoor) / deBoorBu;
end

deBoorDer2 = DeBoorCoxDer2Cal(alfaMatrixDer2,  CtrolPointMulweight, uIndex);
deBoorDer2Bu = DeBoorCoxDer2Cal(alfaMatrixDer2, weightVector, uIndex);

if deBoorBu == 0
    KnotCoorDer2 = [0 0 0];
else
    KnotCoorDer2 = (deBoorDer2 - 2 *deBoorDer1Bu * KnotCoorDer1 - deBoorDer2Bu * KnotCoor) / deBoorBu;
end


DeBoorP(2, :) = KnotCoorDer1;
DeBoorP(3, :) = KnotCoorDer2;

end

%% findSpan函数调用
function index = findSpan(n, p, u, U)

% 利用二分法得到u所在节点向量的区间号
% 输入n,p,u,U。返回所在区间的下标
% n = m - p - 1，m为节点总数-1
% p为要求的基函数阶数
% u为参数
% U为节点向量

if u == U(n + 2)
    index = n;
    return;
end

low = p + 1;
high = n + 2;
mid = floor((low + high) / 2);
 
 while u < U(mid) || u >= U(mid + 1)
     if u < U(mid)
         high = mid;
     else
         low = mid;
     end
     mid = floor((low + high) / 2);
 end
 index = mid - 1;
 
end

%% DeboorCal子函数调用
function P = DeBoorCoxCal( alfaMatrix, CtrlP, uIndex)
% 计算型值点

global curveDegree; % 阶数
n = size(CtrlP, 1);

m = size(CtrlP, 2);
iterativeMatrix = zeros(curveDegree + 1, m,curveDegree + 1);

for temp3 = curveDegree : -1 : 0
    for temp4 = 0 : temp3
        if temp3 == curveDegree
            if uIndex ~= n
                iterativeMatrix(temp4 + 1, :,temp3 + 1) = CtrlP(uIndex - temp3 + temp4 + 1, :);
            else
                if temp4 ~= temp3
                    iterativeMatrix(temp4 + 1, :,temp3 + 1) = CtrlP(uIndex - temp3 + temp4 + 1, :);
                else
                    iterativeMatrix(temp4 + 1, :,temp3 + 1) = zeros(1, m);
                end
            end
        else
            iterativeMatrix(temp4 + 1, :,temp3 + 1) = (1 - alfaMatrix(temp4 + 1, temp3 + 1)) * iterativeMatrix(temp4 + 1, :,temp3 + 2) +...
                alfaMatrix(temp4 + 1, temp3 + 1) * iterativeMatrix(temp4 + 2, :,temp3 + 2);
        end
    end
end

P = iterativeMatrix(1, :, 1);
end

%% DeBoorCoxDer1Cal子函数调用
function derVector = DeBoorCoxDer1Cal(alfaMatrix, CtrlP,uIndex)
% 计算一阶导矢

global curveDegree; % 阶数

m = size(CtrlP, 2);
iteratMatrix = zeros(curveDegree + 1, m,curveDegree + 1);

for temp3 = curveDegree:-1:1
    for temp4 = 1:temp3
        if temp3 == curveDegree
            iteratMatrix(temp4 + 1, :, temp3 + 1) = TempIterative(CtrlP, uIndex - temp3 + temp4);
        else
            iteratMatrix(temp4 + 1, :, temp3 + 1) = (1 - alfaMatrix(temp4 + 1, temp3 + 1)) * iteratMatrix(temp4 + 1, :, temp3 + 2) + ...
                alfaMatrix(temp4 + 1, temp3 + 1) * iteratMatrix(temp4 + 2, :, temp3 + 2);
        end
    end
end

derVector = iteratMatrix(2, :, 2);
end

%% DeBoorCoxDer2Cal子函数调用
function der2 = DeBoorCoxDer2Cal(alfaMatrix, CtrlP,uIndex)
% 计算二阶导矢

global curveDegree;         % 阶数
global KnotVector;

m = size(CtrlP, 2);

iterativeMat = zeros(curveDegree + 1, m, curveDegree + 1);

for temp3 = curveDegree: -1 : 2
    for temp4 = 2 : temp3
        if temp3 == curveDegree
            utemp = (KnotVector(uIndex - temp3 + temp4 + curveDegree) - KnotVector(uIndex - temp3 + temp4 + 1));
            if utemp == 0
                iterativeMat(temp4 + 1, :, temp3 + 1) = zeros(1, m);
            else
                iterativeMat(temp4 + 1, :, temp3 + 1) = (curveDegree - 1) * (TempIterative(CtrlP, uIndex - temp3 + temp4) - TempIterative(CtrlP, uIndex - temp3 + temp4 - 1))...
                    / utemp;
            end
        else
            iterativeMat(temp4 + 1, :, temp3 + 1) = (1 - alfaMatrix(temp4 + 1, temp3 + 1)) * iterativeMat(temp4 + 1, :, temp3 + 2) + alfaMatrix(temp4 + 1, temp3 + 1) * iterativeMat(temp4 + 2, :, temp3 + 2);
        end
    end
end

der2 = iterativeMat(3,:, 3);
end



