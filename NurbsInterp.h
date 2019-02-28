
#include "math.h" 
#include "stdlib.h"

/******************************/
// 声明用于NURBS曲线表达的常量
/******************************/
// 声明节点矢量的维数
#define KNOTVECTORLEN 29        // Fan 曲线
// 声明控制点的个数
#define CONTROLPOINTQ 25	    // Fan 曲线
// 声明NURBS曲线的阶数
#define NURBSORDER 3	       // Fan 曲线


// #define KNOTVECTORLEN 56        // Blade 曲线
// #define CONTROLPOINTQ 52	    // Blade 曲线
// #define NURBSORDER 3	       // Blade 曲线


/******************************/
// 声明利用de-Boor Cox公式计算型值点时用到的全局变量
/******************************/
// 声明计算得到的刀尖点型值点，即Cx，Cy，Cz
extern double xKnotCoor;
extern double yKnotCoor;
extern double zKnotCoor;
// 声明计算得到的刀尖点型值点的一阶导矢，即Cx'，Cy'，Cz'
extern double xKnotCoorDer1;
extern double yKnotCoorDer1;
extern double zKnotCoorDer1;
// 声明计算得到的刀尖点型值点的二阶导矢，即Cx"，Cy"，Cz"
extern double xKnotCoorDer2;
extern double yKnotCoorDer2;
extern double zKnotCoorDer2;
// 声明计算得到的刀尖点型值点的三阶导矢，即Cx"'，Cy"'，Cz"'
extern double xKnotCoorDer3;
extern double yKnotCoorDer3;
extern double zKnotCoorDer3;
// 声明计算得到的刀轴方向型值点，即Ci，Cj，Ck
extern double iKnotCoor;
extern double jKnotCoor;
extern double kKnotCoor;
// 声明xyzijk的控制点，权值以及节点矢量
extern double knotVector[KNOTVECTORLEN];
extern double xCtrlCoor[CONTROLPOINTQ];
extern double yCtrlCoor[CONTROLPOINTQ];
extern double zCtrlCoor[CONTROLPOINTQ];
extern double iCtrlCoor[CONTROLPOINTQ];
extern double jCtrlCoor[CONTROLPOINTQ];
extern double kCtrlCoor[CONTROLPOINTQ];
extern double weightVector[CONTROLPOINTQ];
// 声明w_i*P_i的数值向量
extern 	double xMulWeight[CONTROLPOINTQ];
extern 	double yMulWeight[CONTROLPOINTQ];
extern 	double zMulWeight[CONTROLPOINTQ];
extern 	double iMulWeight[CONTROLPOINTQ];
extern 	double jMulWeight[CONTROLPOINTQ];
extern 	double kMulWeight[CONTROLPOINTQ];

// 利用de-Boor Cox公式计算各型值点以及型值点的导矢
double *DeboorToolTipOrien(double uParameter);	// 用于计算型值点，一二阶导矢，曲率和曲率半径
double DeBoorCoxCal(double alfaMatrix[NURBSORDER][NURBSORDER], double *ctrlPointCor, int nurbsOrder, int uIndex);
double DeBoorCoxDer1Cal(double *knotVector, double alfaMatrix[NURBSORDER][NURBSORDER], double *ctrlPointCor, int nurbsOrder, int uIndex);
double DeBoorCoxDer2Cal(double *knotVector, double alfaMatrix[NURBSORDER][NURBSORDER], double *ctrlPointCor, int nurbsOrder, int uIndex);
double DeBoorCoxDer3Cal(double *knotVector, double *ctrlPointCor, int nurbsOrder, int uIndex);
// 在计算一二阶导矢时，需要计算一个中间迭代式
double TempIterative(double *knotVector, double *ctrlPointCor, int nurbsOrder, int indexInterative);








