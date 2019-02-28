/*=============================*
 * File: ContourErrorCal *
 *============================*/

 /*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  ContourErrorCal
#define S_FUNCTION_LEVEL 2

 /*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */

#include "simstruc.h"
#include "NurbsInterp.h"
#include "math.h"
#include "stdlib.h"
#include "ctype.h"

/*====================*
 * S-function methods *
 *====================*/

/*========================*
g* General Defines/macros *
*========================*/
/* total number of block parameters */
#define NUM_PAR    7

#define NUM_INPUT  2
#define NUM_OUTPUT 3

#define pi  3.1415926535898

// 定义宏变量，用于选择参数曲线
#define CURVE_SELECTION

// 定义指向参数的指针
real_T *refPointValueX     ;
real_T *refPointValueY     ;
real_T *refPointValueZ     ;
real_T *refPointValueI     ;
real_T *refPointValueJ     ;
real_T *refPointValueK     ;
real_T *uNurbs             ;

// 定义变量，保存计算次数
int numOfCal=1;

// 定义变量，保存仿真步长
double SimStep=0.001;

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* Set and Check parameter count  */
    ssSetNumSFcnParams(S, NUM_PAR);  /* Number of expected parameters */
    
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
    {
         /*
         * If the the number of expected input parameters is not equal
         * to the number of parameters entered in the dialog box return.
         *Simulink will generate an error indicating that there is a parameter mismatch.
         */
        return;
    }
    
    /* 
     * Configure tunability of parameters.  By default, all parameters are
     * tunable (changeable) during simulation.  If there are parameters that 
     * cannot change during simulation, such as any parameters that would change 
     * the number of ports on the block, the sample time of the block, or the 
     * data type of a signal, mark these as non-tunable using a call like this:
     * 
     *    ssSetSFcnParamTunable(S, 0, 0);
     *
     * which sets parameter 0 to be non-tunable (0).
     *
     */
    ssSetSFcnParamTunable(S, 0, 0);
    ssSetSFcnParamTunable(S, 1, 0);
    ssSetSFcnParamTunable(S, 2, 0);
    ssSetSFcnParamTunable(S, 3, 0);
    ssSetSFcnParamTunable(S, 4, 0);
    ssSetSFcnParamTunable(S, 5, 0);
    ssSetSFcnParamTunable(S, 6, 0);    
     /*
     * Configure the input ports. First set the number of input ports. 
     */
    if (!ssSetNumInputPorts(S, NUM_INPUT)) 
    {
        return;
    }
    
    /*
     * input 0 is dynamically sized and  typed.
     *   Via data type propagation, Simulink will propose data types for
     *   any ports that are dynamically typed.  Simulink will call the function
     *   mdlSetInputPortDataType or mdlSetOutputPortDataType.  These functions
     *   can accept the proposed data type or refuse it, i.e. error out.  These
     *   functions also have the opportunity to use all currently available
     *   information to set any input or output data types that are still 
     *   dynamically typed.  A typically example is to force an output to
     *   be the same as the data type that was just proposed for an input
     *   (or visa versa).
     */
    ssSetInputPortWidth(             S, 0, 1 );
    ssSetInputPortDataType(          S, 0, DYNAMICALLY_TYPED );
    ssSetInputPortDirectFeedThrough( S, 0, 1 );
    
    /* input 1 is dynamically sized and  typed. */
    ssSetInputPortWidth(             S, 1, 8  );
    ssSetInputPortDataType(          S, 1, DYNAMICALLY_TYPED );
    ssSetInputPortDirectFeedThrough( S, 1, 1 );
    
     /*
     * Configure the output ports. First set the number of output ports.
     */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUT)) 
    {
        return;
    }
    
    /* output 0 is dynamically sized and typed. */
    ssSetOutputPortWidth(    S, 0, 19  );
    ssSetOutputPortDataType( S, 0, DYNAMICALLY_TYPED );
    
    /* output 1 is dynamically sized and typed. */
    ssSetOutputPortWidth(    S, 1, 1  );
    ssSetOutputPortDataType( S, 1, DYNAMICALLY_TYPED );
    
    /* output 2 is dynamically sized and typed. */
    ssSetOutputPortWidth(    S, 2, 1  );
    ssSetOutputPortDataType( S, 2, DYNAMICALLY_TYPED );
    
     /* sample times */
    // Use a positive integer greater than 0 set block-based sample times
    // ssSetNumSampleTimes is used to set the number of sample rate.
    ssSetNumSampleTimes(S, 1);
    
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    /*
     * Set size of the work vectors.
    */
    ssSetNumDWork(S, 19); // number of DWork vectors
    ssSetNumRWork(S, 0); // number of real work vector elements   
    ssSetNumIWork(S, 0); // number of integer work vector elements
    ssSetNumPWork(S, 0); // number of pointers work vector elements
    ssSetNumModes(S, 0); // number of mode work vector elements
    ssSetNumNonsampledZCs(S, 0); // number of nonsampled zero crossings
    
    // Set the widths and data types of the DWork vectors for each axis
    ssSetDWorkWidth(S, 0, 1);
    ssSetDWorkDataType(S, 0, SS_DOUBLE);
    ssSetDWorkWidth(S, 1, 1);
    ssSetDWorkDataType(S, 1, SS_DOUBLE);
    ssSetDWorkWidth(S, 2, 1);
    ssSetDWorkDataType(S, 2, SS_DOUBLE);
    
    ssSetDWorkWidth(S, 3, 1);
    ssSetDWorkDataType(S, 3, SS_DOUBLE);
    ssSetDWorkWidth(S, 4, 1);
    ssSetDWorkDataType(S, 4, SS_DOUBLE);
    ssSetDWorkWidth(S, 5, 1);
    ssSetDWorkDataType(S, 5, SS_DOUBLE);
    
    ssSetDWorkWidth(S, 6, 1);
    ssSetDWorkDataType(S, 6, SS_DOUBLE);
    ssSetDWorkWidth(S, 7, 1);
    ssSetDWorkDataType(S, 7, SS_DOUBLE);
    ssSetDWorkWidth(S, 8, 1);
    ssSetDWorkDataType(S, 8, SS_DOUBLE);
    
    ssSetDWorkWidth(S, 9, 1);
    ssSetDWorkDataType(S, 9, SS_DOUBLE);
    ssSetDWorkWidth(S, 10, 1);
    ssSetDWorkDataType(S, 10, SS_DOUBLE);
    ssSetDWorkWidth(S, 11, 1);
    ssSetDWorkDataType(S, 11, SS_DOUBLE);
    
    ssSetDWorkWidth(S, 12, 1);
    ssSetDWorkDataType(S, 12, SS_DOUBLE);
    ssSetDWorkWidth(S, 13, 1);
    ssSetDWorkDataType(S, 13, SS_DOUBLE);
    ssSetDWorkWidth(S, 14, 1);
    ssSetDWorkDataType(S, 14, SS_DOUBLE);
    
    ssSetDWorkWidth(S, 15, 1);
    ssSetDWorkDataType(S, 15, SS_DOUBLE);
    ssSetDWorkWidth(S, 16, 1);
    ssSetDWorkDataType(S, 16, SS_DOUBLE);
    ssSetDWorkWidth(S, 17, 1);
    ssSetDWorkDataType(S, 17, SS_DOUBLE);    
    
    ssSetDWorkWidth(S, 18, 1);
    ssSetDWorkDataType(S, 18, SS_DOUBLE);    

    /*
     * All options have the form SS_OPTION_<name> and are documented in
     * matlabroot/simulink/include/simstruc.h. The options should be
     * bitwise or'd together as in
     *   ssSetOptions(S, (SS_OPTION_name1 | SS_OPTION_name2))
     */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); // general options (SS_OPTION_xx)

    //初始化数据
    refPointValueX      = mxGetPr(ssGetSFcnParam(S,0));
    refPointValueY      = mxGetPr(ssGetSFcnParam(S,1));
    refPointValueZ      = mxGetPr(ssGetSFcnParam(S,2));
    refPointValueI      = mxGetPr(ssGetSFcnParam(S,3));
    refPointValueJ      = mxGetPr(ssGetSFcnParam(S,4));
    refPointValueK      = mxGetPr(ssGetSFcnParam(S,5));    
    uNurbs              = mxGetPr(ssGetSFcnParam(S,6));
}/* end mdlInitializeSizes */


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.If you specify that you have no sample times, then
 *    the S-function is assumed to have one inherited sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
 /*  The sample times are specified as pairs "[sample_time, offset_time]"
 *    via the following macros:
 *      ssSetSampleTime(S, sampleTimePairIndex, sample_time)
 *      ssSetOffsetTime(S, offsetTimePairIndex, offset_time)
 *    Where sampleTimePairIndex starts at 0.
 */
    
    /* Register one pair for each sample time */
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}/* end mdlInitializeSampleTimes */ 


#define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ===========================================
 * Abstract:
 *    Initialize both continuous states to zero
 */
static void mdlInitializeConditions(SimStruct *S)
{
    // 定义变量，保存工作向量内容       
    real_T *ContourErrorXLastFirst  = (real_T*) ssGetDWork(S,0);
    real_T *ContourErrorXLastSecond = (real_T*) ssGetDWork(S,1);    
    real_T *ContourErrorXLastThird  = (real_T*) ssGetDWork(S,2);
    
    real_T *ContourErrorYLastFirst  = (real_T*) ssGetDWork(S,3);
    real_T *ContourErrorYLastSecond = (real_T*) ssGetDWork(S,4);    
    real_T *ContourErrorYLastThird  = (real_T*) ssGetDWork(S,5);

    real_T *ContourErrorZLastFirst  = (real_T*) ssGetDWork(S,6);
    real_T *ContourErrorZLastSecond = (real_T*) ssGetDWork(S,7);    
    real_T *ContourErrorZLastThird  = (real_T*) ssGetDWork(S,8);

    real_T *ContourErrorILastFirst  = (real_T*) ssGetDWork(S,9);
    real_T *ContourErrorILastSecond = (real_T*) ssGetDWork(S,10);    
    real_T *ContourErrorILastThird  = (real_T*) ssGetDWork(S,11);
    
    real_T *ContourErrorJLastFirst  = (real_T*) ssGetDWork(S,12);
    real_T *ContourErrorJLastSecond = (real_T*) ssGetDWork(S,13);    
    real_T *ContourErrorJLastThird  = (real_T*) ssGetDWork(S,14);    

    real_T *ContourErrorKLastFirst  = (real_T*) ssGetDWork(S,15);
    real_T *ContourErrorKLastSecond = (real_T*) ssGetDWork(S,16);    
    real_T *ContourErrorKLastThird  = (real_T*) ssGetDWork(S,17);
    
    real_T *FootPointCLast  = (real_T*) ssGetDWork(S,18);  

    /*  Initialize the DWork vectors */
    ContourErrorXLastFirst[0]  = 0.0;
    ContourErrorXLastSecond[0] = 0.0;
    ContourErrorXLastThird[0]  = 0.0;

    ContourErrorYLastFirst[0]  = 0.0;
    ContourErrorYLastSecond[0] = 0.0;
    ContourErrorYLastThird[0]  = 0.0;

    ContourErrorZLastFirst[0]  = 0.0;
    ContourErrorZLastSecond[0] = 0.0;
    ContourErrorZLastThird[0]  = 0.0;

    ContourErrorILastFirst[0]  = 0.0;
    ContourErrorILastSecond[0] = 0.0;
    ContourErrorILastThird[0]  = 0.0;

    ContourErrorJLastFirst[0]  = 0.0;
    ContourErrorJLastSecond[0] = 0.0;
    ContourErrorJLastThird[0]  = 0.0;
    
    ContourErrorKLastFirst[0]  = 0.0;
    ContourErrorKLastSecond[0] = 0.0;
    ContourErrorKLastThird[0]  = 0.0;
    
    FootPointCLast[0]=0.0;
}


#define MDL_START
/* Function: mdlStart ==========================================================
 * Abstract:
 *      Here we cache the state (true/false) of the XDATAEVENLYSPACED parameter.
 *      We do this primarily to illustrate how to "cache" parameter values (or
 *      information which is computed from parameter values) which do not change
 *      for the duration of the simulation (or in the generated code). In this
 *      case, rather than repeated calls to mxGetPr, we save the state once.
 *      This results in a slight increase in performance.
 */
static void mdlStart(SimStruct *S)
{
} /*  MDL_START */


double  *DeboorToolTipOrien(double uParameter)
{
     // 定义计算得到的刀尖点型值点，即Cx，Cy，Cz
    double xKnotCoor=0;
    double yKnotCoor=0;
    double zKnotCoor=0;
    // 定义计算得到的刀尖点型值点的一阶导矢，即Cx'，Cy'，Cz'
    double xKnotCoorDer1=0;
    double yKnotCoorDer1=0;
    double zKnotCoorDer1=0;
    // 定义计算得到的刀尖点型值点的二阶导矢，即Cx"，Cy"，Cz"
    double xKnotCoorDer2=0;
    double yKnotCoorDer2=0;
    double zKnotCoorDer2=0;
    // 定义计算得到的刀尖点型值点的三阶导矢，即Cx"'，Cy"'，Cz"'
    double xKnotCoorDer3=0;
    double yKnotCoorDer3=0;
    double zKnotCoorDer3=0; 
    
    // 定义计算得到的刀轴矢量型值点，即Ci，Cj，Ck
    double iKnotCoor=0;
    double jKnotCoor=0;
    double kKnotCoor=0;    
    
    // 定义xyz的控制点，权值以及节点矢量
    #ifdef CURVE_SELECTION
    // 定义节点矢量，fan曲线
    double knotVector[KNOTVECTORLEN]={0,0,0,0,0.106505793441501,0.157284527891952,0.199426140710703,0.234151035701386,0.266827885254851,0.300976958184472,\
    0.340545525832759,0.387257035050466,0.433031806140371,0.471312812708211,0.498755113378692,0.523379024673771,0.546560267841075,0.571456154181000,\
    0.601575236029666,0.644990504057629,0.695759844487100,0.749760773458549,0.799359778440787,0.849103852157646,0.899353829759719,1,1,1,1};

    // 定义控制点的x轴坐标
    double xCtrlCoor[CONTROLPOINTQ]={113.560775000000,117.907784172815,121.462357619208,106.012714741771,96.1401560617727,89.1911507101181,79.7798213582500,\
    71.6724064108289,64.9824828874336,54.8424637547536,39.2392540175040,32.8933055388342,26.4490644896035,22.4116145178198,18.2736073139743,16.6339019865314,\
    16.3334292670881,21.2856317933606,27.4817364686037,29.4722901725788,23.6369028383090,6.32107092312859,-18.9344035716273,-38.6283247647271,\
    -49.4388780000000};

    // 定义控制点的y轴坐标
    double yCtrlCoor[CONTROLPOINTQ]={7.73526600000000,-3.72550226174048,-27.3277457365089,-54.8011679321253,-64.2516317460606,-66.4558822681257,\
    -65.6471075625930,-61.0876209490450,-53.9448519849804,-39.2705965087025,-23.8067759915909,-18.8923033586951,-16.6641134539066,-16.1633900205040,\
    -17.6184300123742,-21.6391152140663,-28.6880626467441,-39.9762303990255,-68.3782350657572,-85.5601869775015,-105.212470043406,-115.316836001493,\
    -120.008716617807,-115.952845290824,-108.784390000000};

    // 定义控制点的z轴坐标
    double zCtrlCoor[CONTROLPOINTQ]={-2.20931400000000,-1.40854242697360,-0.0607452135072286,2.65001386114913,3.34041259418642,6.37847216653963,\
    7.21905148949902,6.92801120194676,5.95234207426767,4.96872130112414,3.57701522569175,3.19929368574463,2.41772678760589,2.22552494327216,\
    0.164416924317616,-1.71809989263841,-2.92674119711899,-3.69925250451544,-5.59355790378744,-6.82029075680885,-4.51969660571512,-1.66864654212492,\
    -0.789248427751609,1.11844292823757,2.08953700000000};

    // 定义控制点的i轴坐标
    double iCtrlCoor[CONTROLPOINTQ]={-0.107258000000000,-0.0442951092811055,0.0883079549974777,0.255640113370598,0.319849384790067,0.332112955148523,\
    0.328228461139330,0.314282212419524,0.287314050368324,0.217101666751941,0.133752319337668,0.105762867103592,0.0958621942338097,0.0955689665011038,\
    0.109296691070313,0.135314310549536,0.182196255585854,0.250997719555162,0.407132858316953,0.482342243765666,0.572817263839144,0.635725663040383,\
    0.665569458879872,0.652940777489441,0.618930000000000};

    // 定义控制点的j轴坐标
    double jCtrlCoor[CONTROLPOINTQ]={0.624902000000000,0.650594140264470,0.677305960087371,0.606137531865309,0.558962387909486,0.515187098751786,\
    0.476417021152194,0.443317779871489,0.411783154525997,0.363299111772655,0.270260937384927,0.228812371657891,0.186771029028180,0.159753146439207,\
    0.134903724250184,0.125093956726947,0.125287778985102,0.162950769533927,0.206318787249968,0.212519308787444,0.194450982679685,0.0996377097838548,\
    -0.0409186160934884,-0.156260018255702,-0.223905000000000};

    // 定义控制点的k轴坐标
    double kCtrlCoor[CONTROLPOINTQ]={0.773300000000000,0.761794011543888,0.741771672576981,0.755716854777584,0.765688250458093,0.790582915830076,\
    0.816045170951245,0.839806542940226,0.865513471260849,0.909383265643032,0.954749752480179,0.968020581204941,0.977914337378161,0.982633502412545,\
    0.984923801700219,0.983069457691021,0.975714519219553,0.957165187529715,0.893753675562993,0.850956283469529,0.798998343425429,0.768231681217848,\
    0.751318956770314,0.745264736177389,0.752856000000000};

    // 定义权值向量
    double weightVector[CONTROLPOINTQ]={1.0000,1.0000,1.0000,1.000,1.0000,1.0000,1.0000,1.0000,\
        1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000};
    #else
    // 定义节点矢量，blade曲线    
    double knotVector[KNOTVECTORLEN]={0,0,0,0,0.0408270951965416,0.0611940355785803,0.0815127102812400,0.0998068519177771,0.116168475013227,0.132453051992392,0.150681786178957,\
    0.170642433374905,0.190571907488079,0.210343983298041,0.230062487410986,0.249627968419188,0.269109785813314,0.288533011464667,0.307925663610566,0.327310086207321,\
    0.346889505719849,0.366471914550031,0.386108414230578,0.405539455343359,0.424931658074850,0.444196974894897,0.466147053560963,0.490188087009663,0.514656518219301,\
    0.536800316606819,0.557236016663954,0.577690222259982,0.596120480446577,0.612792136648440,0.629402593414203,0.648008564290944,0.668344678661991,0.688679818231071,\
    0.708993354905670,0.729278370921988,0.749533326892922,0.769747806784225,0.789916342472583,0.810036746374911,0.830119399259965,0.850162193357179,0.870171676526078,\
    0.890158495621789,0.910154164943106,0.928451988082027,0.944847257783646,0.961315507500795,1,1,1,1};    
    
    // 定义控制点的x轴坐标
    double xCtrlCoor[CONTROLPOINTQ]={74.9684000000000,72.7433531113297,69.4637463440642,65.0564602973189,62.0648681491990,58.9474230399046,57.3738067915712,55.7673603964895,53.0803170932839,\
     50.7515655695079,48.5505919169795,46.5125171958761,44.7297238106929,43.1179294396447,41.6580862419665,40.3504440354397,38.8790602860284,37.3158823163231,\
     35.6127949206256,33.6495185413456,31.6763321538687,29.5259515641397,27.2688029982275,24.9715958788479,21.5845853639681,22.0068232495600,27.5110221341699,\
     29.7846400053149,32.3695636363164,34.4157367449989,36.5457493551781,37.5615231483619,38.6810504405495,40.4307826897082,41.9434919544858,43.4477562029341,\
     44.7722668475837,46.1722217939660,47.7101667299969,49.4107495030916,51.2874592271498,53.3695196600161,55.6645270953027,58.1734642262872,60.8758797313934,\
     63.7863172857487,66.6480873973455,69.8035615330687,71.6947188416029,74.1158188738777,78.1522289421750,79.8596999999999};    
    
    // 定义控制点的y轴坐标
    double yCtrlCoor[CONTROLPOINTQ]={-35.1178000000000,-34.8449904292686,-34.3985591498241,-33.5690570175588,-33.0342605321579,-32.5949922401735,-32.4319262520343,-32.2715461751259,\
     -32.1634418342067,-32.2698950810537,-32.5775550234464,-33.1226257540289,-33.8839286347776,-34.8703154043207,-36.0495171416578,-37.4346995857198,-38.8733536038383,\
     -40.2664486975342,-41.6824247242068,-43.1790051529728,-44.5375592011837,-45.8770977141321,-47.1212683303803,-48.2749153056296,-49.6424849960614,-49.6136149680830,\
     -46.5831227724961,-45.0382700545476,-43.2275434747532,-41.6564737096269,-39.8686957673871,-38.9430605455226,-37.9015078068679,-36.1713236050619,-34.5620136751747,\
     -32.8858549591366,-31.2413829542469,-29.7703772135650,-28.5319198684234,-27.5230010502209,-26.7494978662432,-26.2090240771741,-25.8998857343956,-25.8134276114032,\
     -25.9169450066287,-26.2005192296588,-26.6114501535619,-27.1524155150160,-27.4216594623506,-27.7416473129740,-28.1852301220492,-28.3110000000000};
    
    // 定义控制点的z轴坐标
    double zCtrlCoor[CONTROLPOINTQ]={112.894600000000,113.279790703642,113.913163717178,114.699161423338,115.515458370962,116.850849212101,117.703160809429,118.657227373733,120.565262603178,\
     122.634588917956,124.915787414865,127.388016145654,129.861359852539,132.353960697068,134.815043297329,137.227902527975,139.493660685232,141.759416735934,143.842064287883,\
     145.973984984107,147.838107352694,149.690958878137,151.284384493880,153.241065205788,153.493021454625,162.602594082611,158.803277097218,157.532151167653,155.595250167404,\
     153.843128542818,151.672240823165,150.535479820709,149.172632353644,146.872410336341,144.575069417132,142.034141386636,139.425683915361,136.753809997969,134.053979007558,\
     131.367055839086,128.745063406684,126.236803006082,123.898728339281,121.781030330285,119.959264781041,118.454864236079,117.393673060959,116.668368613364,116.215488535698,\
     115.665622503005,114.803170916957,114.507500000000};    
    
    // 定义控制点的i轴坐标
    double iCtrlCoor[CONTROLPOINTQ]={-0.0371769000000000,0.0170380674632655,0.0879546447732690,0.160975636086260,0.205511358539013,0.247983363644893,0.268439075532751,\
    0.289288442089612,0.322267498673386,0.348685100795015,0.372706545897017,0.394535520335839,0.413011120975195,0.428011736751492,0.440333337642362,0.450419950109937,\
    0.459228773035920,0.465944539031994, 0.470268176284741,0.472874503188417,0.473994732654784,0.472447631175639,0.466883735948654,0.457246329880201,0.435562292111987,\
    0.425842935072119,0.443146623893145,0.448159609287062,0.449335904413722,0.447936418961415,0.446575801658604,0.446621439872204,0.446001191086250,0.443513533475304,\
    0.438187337487115,0.429814260113480,0.420213030057205,0.408509176248067,0.394126639090020,0.376184480931322,0.355137904534941,0.331397499320037,0.304011162516936,\
    0.271867787723268,0.234916807888040,0.193304922675324,0.149538634267047,0.0950716453026715,0.0584848507233034,0.00883875625103607,-0.0820851517274831,-0.146135499999998};
    
    // 定义控制点的j轴坐标
    double jCtrlCoor[CONTROLPOINTQ]={-0.339332600000000,-0.357893510296783,-0.391401663797343,-0.440376948850493,-0.474662436591595,-0.511591471852472,-0.530527013098082,-0.550026678356929,-0.583060394766143,\
    -0.612433359706928,-0.641032044569005,-0.668504700320251,-0.693707473645825,-0.717453564798669,-0.739659063772372,-0.760373523816635,-0.780798191870014,-0.800344650439839,\
    -0.818350404646073,-0.835582166635550,-0.849992694650535,-0.863824320695744,-0.876328869671727,-0.888133212059814,-0.900740883494064,-0.905041261726591,-0.895194336473287,\
    -0.885527233597311,-0.875321529335834,-0.865385964677575,-0.851331126845022,-0.842556544748487,-0.832478771727383,-0.814585875067413,-0.796644140767197,-0.776676729878733,\
    -0.756902286483513,-0.736018877998970,-0.713703766097411,-0.689947692201334,-0.664512052907538,-0.637378996036611,-0.608471057812262,-0.577720975091296,-0.545161568461691,\
    -0.511254200873836,-0.479270438448765,-0.446299202410541,-0.428050566931625,-0.405623967239053,-0.374439470532540,-0.362292100000000};
    
    // 定义控制点的k轴坐标
    double kCtrlCoor[CONTROLPOINTQ]={0.939931500000000,0.935333601504799,0.918304048854475,0.884055187856583,0.856763482330372,0.823128752748557,0.804167733835501,0.783910835944571,0.746621675497071,\
    0.710103364588403,0.671676363377355,0.631156842989595,0.590782366857176,0.550321614720925,0.509655067793236,0.468740285478461,0.424632544221207,0.378387042134942,\
    0.331743419840355,0.281274629806753,0.231845616332287,0.178204213263989,0.122170299556385,0.0651831223457880,-0.0207832171789117,-0.00766809769615049,0.0645581760136915,\
    0.126686982503720,0.180887043083907,0.226703971839568,0.276183386928234,0.301284428222239,0.329512823091442,0.374962823864362,0.417249854422740,0.461299977414493,\
    0.501193416370392,0.540484441728402,0.579713982669619,0.619114397348263,0.658175248528272,0.696327902869043,0.733750408698763,0.770377333106787,0.805556889997077,\
    0.838128569769232,0.865756726877767,0.890390580084403,0.902042844572467,0.915178099389819,0.926934640171849,0.920537300000000};
    
    // 定义权值向量
    double weightVector[CONTROLPOINTQ]={1.0000,1.0000,1.0000,1.000,1.0000,1.0000,1.0000,1.0000,\
    1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,\
    1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,\
    1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000};
    #endif

    // 定义控制点和权值的乘积
    double xMulWeight[CONTROLPOINTQ]={0};
    double yMulWeight[CONTROLPOINTQ]={0};
    double zMulWeight[CONTROLPOINTQ]={0};
    double iMulWeight[CONTROLPOINTQ]={0};
    double jMulWeight[CONTROLPOINTQ]={0};
    double kMulWeight[CONTROLPOINTQ]={0};
    
    // 定义用于循环计算时的临时索引变量
    int tempLoopIndex=0;   
    
    // 定义nurbs曲线的次数
    int nurbsOrder=NURBSORDER;
    // 定义uParameter在节点矢量向量中所处的index
    int uIndex=0;

    // 定义临时变量
    int tempData;
    int tempData2;
    // 定义在利用DeBoorCox公式进行计算时A(u)和B(u)，以及其导数
    double xdeBoorAu=0;
    double ydeBoorAu=0;
    double zdeBoorAu=0;
    double ideBoorAu=0;
    double jdeBoorAu=0;
    double kdeBoorAu=0;    
    double deBoorBu=0;

    double xdeBoorAuDer1=0;
    double ydeBoorAuDer1=0;
    double zdeBoorAuDer1=0;
    double deBoorBuDer1=0;

    double xdeBoorAuDer2=0;
    double ydeBoorAuDer2=0;
    double zdeBoorAuDer2=0;
    double deBoorBuDer2=0;
    
    double xdeBoorAuDer3=0;
    double ydeBoorAuDer3=0;
    double zdeBoorAuDer3=0;
    double deBoorBuDer3=0;

    // 定义采用de-Boor Cox公式计算型值点以及型值点的一二阶导矢时
    // 所用到的系数矩阵alfaMatrix[2][2]
    double alfaMatrix[NURBSORDER][NURBSORDER]={0};
    double alfaMatrixDer1[NURBSORDER][NURBSORDER]={0};	
    double alfaMatrixDer2[NURBSORDER][NURBSORDER]={0};
    
    // 定义数组，保存计算得到的刀尖点型值点、一二三阶导失和刀轴矢量型值点
    static double DeboorCoxDerCal[15]={0};
    
    // 初始化NURBS计算中的变量
    for(tempLoopIndex=0;tempLoopIndex<CONTROLPOINTQ;tempLoopIndex++)
    {
        xMulWeight[tempLoopIndex]=xCtrlCoor[tempLoopIndex]*weightVector[tempLoopIndex];
        yMulWeight[tempLoopIndex]=yCtrlCoor[tempLoopIndex]*weightVector[tempLoopIndex];
        zMulWeight[tempLoopIndex]=zCtrlCoor[tempLoopIndex]*weightVector[tempLoopIndex];
        iMulWeight[tempLoopIndex]=iCtrlCoor[tempLoopIndex]*weightVector[tempLoopIndex];
        jMulWeight[tempLoopIndex]=jCtrlCoor[tempLoopIndex]*weightVector[tempLoopIndex];
        kMulWeight[tempLoopIndex]=kCtrlCoor[tempLoopIndex]*weightVector[tempLoopIndex];        
    } 

    // step 1: 判断uNubs所属的区间范围,即确定index值
    while((uParameter>=knotVector[uIndex+1]) && (uIndex+1<=KNOTVECTORLEN-1)) uIndex++;   

    // step 2: 计算各坐标轴型值点的系数矩阵
    // 该矩阵在以下计算型值点、型值点的一二阶导矢时均会使用
    for(tempData=0;tempData<NURBSORDER;tempData++)
    {
        for(tempData2=0;tempData2<=tempData;tempData2++)
        {
            // tempData对应的是列，tempData2对应的是行
            alfaMatrix[tempData2][tempData]=(uParameter-knotVector[uIndex-tempData+tempData2])/(knotVector[uIndex+1+tempData2]-knotVector[uIndex-tempData+tempData2]);
        }
    }

    // 由于在计算一阶导矢的时候，迭代仅需迭代到P[1][1]，因此，alfa矩阵只需要算除了第一行之外的下三角即可
    for(tempData=NURBSORDER-1;tempData>=1;tempData--)
    {
        for(tempData2=tempData;tempData2>=1;tempData2--)
        {
            // tempData对应的是列，tempData2对应的是行
            alfaMatrixDer1[tempData2][tempData]=(uParameter-knotVector[uIndex-tempData+tempData2])/(knotVector[uIndex+tempData2]-knotVector[uIndex-tempData+tempData2]);
        }
    }

    // 由于在计算二阶导矢的时候，迭代仅需迭代到P[2][2]，因此，alfa矩阵只需要算除了第一、二行之外的下三角即可
    for(tempData=NURBSORDER-1;tempData>=2;tempData--)
    {
        for(tempData2=tempData;tempData2>=2;tempData2--)
        {
            // tempData对应的是列，tempData2对应的是行
            alfaMatrixDer2[tempData2][tempData]=(uParameter-knotVector[uIndex-tempData+tempData2])/(knotVector[uIndex+tempData2-1]-knotVector[uIndex-tempData+tempData2]);
        }
    }

    // step 3: 计算各坐标轴型值点
    xdeBoorAu=DeBoorCoxCal(alfaMatrix, xMulWeight, nurbsOrder, uIndex);
    ydeBoorAu=DeBoorCoxCal(alfaMatrix, yMulWeight, nurbsOrder, uIndex);
    zdeBoorAu=DeBoorCoxCal(alfaMatrix, zMulWeight, nurbsOrder, uIndex);
    ideBoorAu=DeBoorCoxCal(alfaMatrix, iMulWeight, nurbsOrder, uIndex);
    jdeBoorAu=DeBoorCoxCal(alfaMatrix, jMulWeight, nurbsOrder, uIndex);
    kdeBoorAu=DeBoorCoxCal(alfaMatrix, kMulWeight, nurbsOrder, uIndex);    
    deBoorBu=DeBoorCoxCal(alfaMatrix, weightVector, nurbsOrder, uIndex);
    // 以下是各坐标点的坐标值
    if(deBoorBu==0)
    {
        xKnotCoor=0;
        yKnotCoor=0;
        zKnotCoor=0;
        iKnotCoor=0;
        jKnotCoor=0;
        kKnotCoor=0;
    }
    else
    {
        xKnotCoor=xdeBoorAu/deBoorBu;
        yKnotCoor=ydeBoorAu/deBoorBu;
        zKnotCoor=zdeBoorAu/deBoorBu;
        iKnotCoor=ideBoorAu/deBoorBu;
        jKnotCoor=jdeBoorAu/deBoorBu;
        kKnotCoor=kdeBoorAu/deBoorBu;
    }

    // step 4: 计算各坐标轴型值点的一二三阶导矢
    // 首先计算一阶导矢
    xdeBoorAuDer1=DeBoorCoxDer1Cal(knotVector, alfaMatrixDer1, xMulWeight, nurbsOrder, uIndex);
    ydeBoorAuDer1=DeBoorCoxDer1Cal(knotVector, alfaMatrixDer1, yMulWeight, nurbsOrder, uIndex);
    zdeBoorAuDer1=DeBoorCoxDer1Cal(knotVector, alfaMatrixDer1, zMulWeight, nurbsOrder, uIndex);
    deBoorBuDer1=DeBoorCoxDer1Cal(knotVector, alfaMatrixDer1, weightVector, nurbsOrder, uIndex);
    // 以下是各坐标点坐标值的一阶导矢
    if(deBoorBu==0)
    {
        xKnotCoorDer1=0;
        yKnotCoorDer1=0;
        zKnotCoorDer1=0;
    }
    else
    {
        xKnotCoorDer1=(xdeBoorAuDer1-deBoorBuDer1*xKnotCoor)/deBoorBu;
        yKnotCoorDer1=(ydeBoorAuDer1-deBoorBuDer1*yKnotCoor)/deBoorBu;	
        zKnotCoorDer1=(zdeBoorAuDer1-deBoorBuDer1*zKnotCoor)/deBoorBu;
    }

    // 计算二阶导矢
    xdeBoorAuDer2=DeBoorCoxDer2Cal(knotVector, alfaMatrixDer2, xMulWeight, nurbsOrder, uIndex);
    ydeBoorAuDer2=DeBoorCoxDer2Cal(knotVector, alfaMatrixDer2, yMulWeight, nurbsOrder, uIndex);
    zdeBoorAuDer2=DeBoorCoxDer2Cal(knotVector, alfaMatrixDer2, zMulWeight, nurbsOrder, uIndex);
    deBoorBuDer2=DeBoorCoxDer2Cal(knotVector, alfaMatrixDer2, weightVector, nurbsOrder, uIndex);
    // 以下是各坐标点坐标值的二阶导矢
    if(deBoorBu==0)
    {
        xKnotCoorDer2=0;
        yKnotCoorDer2=0;
        zKnotCoorDer2=0;
    }
    else
    {
        xKnotCoorDer2=(xdeBoorAuDer2-2*deBoorBuDer1*xKnotCoorDer1-deBoorBuDer2*xKnotCoor)/deBoorBu;
        yKnotCoorDer2=(ydeBoorAuDer2-2*deBoorBuDer1*yKnotCoorDer1-deBoorBuDer2*yKnotCoor)/deBoorBu;
        zKnotCoorDer2=(zdeBoorAuDer2-2*deBoorBuDer1*zKnotCoorDer1-deBoorBuDer2*zKnotCoor)/deBoorBu;
    }
 
    // 计算三阶导矢
    xdeBoorAuDer3=DeBoorCoxDer3Cal(knotVector, xMulWeight, nurbsOrder, uIndex);
    ydeBoorAuDer3=DeBoorCoxDer3Cal(knotVector, yMulWeight, nurbsOrder, uIndex);
    zdeBoorAuDer3=DeBoorCoxDer3Cal(knotVector, zMulWeight, nurbsOrder, uIndex);
    deBoorBuDer3=DeBoorCoxDer3Cal(knotVector,  weightVector, nurbsOrder, uIndex);
    // 以下是各坐标点坐标值的三阶导矢
    if(deBoorBu==0)
    {
        xKnotCoorDer3=0;
        yKnotCoorDer3=0;
        zKnotCoorDer3=0;
    }
    else
    {
        xKnotCoorDer3=((xdeBoorAuDer3-3*deBoorBuDer2*xKnotCoorDer1-2*deBoorBuDer1*xKnotCoorDer2-deBoorBuDer3*xKnotCoor)*deBoorBu-
            deBoorBuDer1*(xdeBoorAuDer2 - 2 *deBoorBuDer1 * xKnotCoorDer1 - deBoorBuDer2 * xKnotCoor))/(deBoorBu*deBoorBu);
        yKnotCoorDer3=((ydeBoorAuDer3-3*deBoorBuDer2*yKnotCoorDer1-2*deBoorBuDer1*yKnotCoorDer2-deBoorBuDer3*yKnotCoor)*deBoorBu-
            deBoorBuDer1*(ydeBoorAuDer2 - 2 *deBoorBuDer1 * yKnotCoorDer1 - deBoorBuDer2 * yKnotCoor))/(deBoorBu*deBoorBu);
        zKnotCoorDer3=((zdeBoorAuDer3-3*deBoorBuDer2*zKnotCoorDer1-2*deBoorBuDer1*zKnotCoorDer2-deBoorBuDer3*zKnotCoor)*deBoorBu-
            deBoorBuDer1*(zdeBoorAuDer2 - 2 *deBoorBuDer1 * zKnotCoorDer1 - deBoorBuDer2 * zKnotCoor))/(deBoorBu*deBoorBu);
    }
    
    DeboorCoxDerCal[0]=xKnotCoor;
    DeboorCoxDerCal[1]=yKnotCoor;
    DeboorCoxDerCal[2]=zKnotCoor;
    DeboorCoxDerCal[3]=xKnotCoorDer1;
    DeboorCoxDerCal[4]=yKnotCoorDer1;
    DeboorCoxDerCal[5]=zKnotCoorDer1;
    DeboorCoxDerCal[6]=xKnotCoorDer2;
    DeboorCoxDerCal[7]=yKnotCoorDer2;
    DeboorCoxDerCal[8]=zKnotCoorDer2;
    DeboorCoxDerCal[9]=xKnotCoorDer3;
    DeboorCoxDerCal[10]=yKnotCoorDer3;
    DeboorCoxDerCal[11]=zKnotCoorDer3;
    DeboorCoxDerCal[12]=iKnotCoor;
    DeboorCoxDerCal[13]=jKnotCoor;
    DeboorCoxDerCal[14]=kKnotCoor;    
    
    return DeboorCoxDerCal;
}


// 利用de-Boor Cox公式计算各型值点以及型值点的导矢
// 以下函数用于计算型值点
double DeBoorCoxCal(double alfaMatrix[NURBSORDER][NURBSORDER], double *ctrlPointCor, int nurbsOrder, int uIndex)
{
    // 定义采用de-Boor Cox公式计算型值点以及型值点的一二阶导矢时
    // 迭代计算结果存放的矩阵iterativeMatrix[3][3]
    double iterativeMatrix[NURBSORDER+1][NURBSORDER+1]={0};
    // 定义两个临时变量，用于迭代时作为迭代行列使用
    // tempData3代表列数
    int tempData3=0;
    // tempData4代表行数
    int tempData4=0;

    // 分别计算各列的数值
    for(tempData3=NURBSORDER; tempData3>=0; tempData3--)
    {
        // 分别计算某列各行的数值
        for(tempData4=0; tempData4<=tempData3; tempData4++)
        {
            if(tempData3==NURBSORDER)
            {
                iterativeMatrix[tempData4][tempData3]= *(ctrlPointCor+uIndex-tempData3+tempData4);
            }
            else
            {
                iterativeMatrix[tempData4][tempData3]=(1-alfaMatrix[tempData4][tempData3]) * iterativeMatrix[tempData4][tempData3+1] + 
                    alfaMatrix[tempData4][tempData3] * iterativeMatrix[tempData4+1][tempData3+1];
            }
        }
    }

    return iterativeMatrix[0][0];
}

// 以下函数用于计算型值点的一阶导矢
double DeBoorCoxDer1Cal(double *knotVector, double alfaMatrixDer1[NURBSORDER][NURBSORDER], double *ctrlPointCor, int nurbsOrder, int uIndex)
{
    // 定义采用de-Boor Cox公式计算型值点以及型值点的一二阶导矢时
    // 迭代计算结果存放的矩阵iterativeMatrix[3][3]
    double iterativeMatrix[NURBSORDER+1][NURBSORDER+1]={0};
    // 定义两个临时变量，用于迭代时作为迭代行列使用
    int tempData3=0;
    int tempData4=0;

    for(tempData3=NURBSORDER;tempData3>=1;tempData3--)
    {
        for(tempData4=1; tempData4<=tempData3; tempData4++)
        {
            if(tempData3==NURBSORDER)
            {  
                iterativeMatrix[tempData4][tempData3]= TempIterative(knotVector,ctrlPointCor,nurbsOrder,uIndex-tempData3+tempData4);
            }
            else
            {
                iterativeMatrix[tempData4][tempData3]=(1-alfaMatrixDer1[tempData4][tempData3]) * iterativeMatrix[tempData4][tempData3+1] + 
                    alfaMatrixDer1[tempData4][tempData3] * iterativeMatrix[tempData4+1][tempData3+1];
            }
        }
    }

    return iterativeMatrix[1][1];
}

// 以下函数用于计算型值点的二阶导矢
double DeBoorCoxDer2Cal(double *knotVector, double alfaMatrixDer2[NURBSORDER][NURBSORDER], double *ctrlPointCor, int nurbsOrder, int uIndex)
{
    // 定义采用de-Boor Cox公式计算型值点以及型值点的一二阶导矢时
    // 迭代计算结果存放的矩阵iterativeMatrix[3][3]
    double iterativeMatrix[NURBSORDER+1][NURBSORDER+1]={0};
    // 定义两个临时变量，用于迭代时作为迭代行列使用
    int tempData3=0;
    int tempData4=0;

    for(tempData3=NURBSORDER;tempData3>=2;tempData3--)
    {
        for(tempData4=2; tempData4<=tempData3; tempData4++)
        {
            if(tempData3==NURBSORDER)
            {
                iterativeMatrix[tempData4][tempData3]= (NURBSORDER-1)*(TempIterative(knotVector,ctrlPointCor,NURBSORDER,uIndex-tempData3+tempData4)-
                    TempIterative(knotVector,ctrlPointCor,NURBSORDER,uIndex-tempData3+tempData4-1))/(*(knotVector+uIndex-tempData3+tempData4+nurbsOrder-1) - *(knotVector+uIndex-tempData3+tempData4));
            }
            else
            {
                iterativeMatrix[tempData4][tempData3]=(1-alfaMatrixDer2[tempData4][tempData3]) * iterativeMatrix[tempData4][tempData3+1] + 
                    alfaMatrixDer2[tempData4][tempData3] * iterativeMatrix[tempData4+1][tempData3+1];
            }
        }
    }

    return iterativeMatrix[2][2];
}

// 以下函数用于计算型值点的三阶导矢
double DeBoorCoxDer3Cal(double *knotVector, double *ctrlPointCor, int nurbsOrder, int uIndex)
{
    // 定义采用de-Boor Cox公式计算型值点以及型值点的一二三阶导矢时
    // 迭代计算结果存放的矩阵iterativeMatrix[3][3]
    double iterativeMatrix[NURBSORDER+1][NURBSORDER+1]={0};
    // 定义两个临时变量，用于迭代时作为迭代行列使用
    int tempData3=0;
    int tempData4=0;

    for(tempData3=NURBSORDER;tempData3>=3;tempData3--)
    {
        for(tempData4=3; tempData4<=tempData3; tempData4++)
        {
            if(tempData3==NURBSORDER)
            {
                iterativeMatrix[tempData4][tempData3]= (NURBSORDER-2)*((NURBSORDER-1)*(TempIterative(knotVector,ctrlPointCor,NURBSORDER,uIndex-tempData3+tempData4)-
                    TempIterative(knotVector,ctrlPointCor,NURBSORDER,uIndex-tempData3+tempData4-1))/(*(knotVector+uIndex-tempData3+tempData4+nurbsOrder-1) - *(knotVector+uIndex-tempData3+tempData4))-
                    (NURBSORDER-1)*(TempIterative(knotVector,ctrlPointCor,NURBSORDER,uIndex-tempData3+tempData4-1)-TempIterative(knotVector,ctrlPointCor,NURBSORDER,uIndex-tempData3+tempData4-2))/
                    (*(knotVector+uIndex-tempData3+tempData4+nurbsOrder-2) - *(knotVector+uIndex-tempData3+tempData4-1)))/
                    (*(knotVector+uIndex-tempData3+tempData4+nurbsOrder-2) - *(knotVector+uIndex-tempData3+tempData4));
            }
        }
    }
    return iterativeMatrix[3][3];
}

// 在计算一二阶导矢时，需要计算一个中间迭代式
double TempIterative(double *knotVector, double *ctrlPointCor, int nurbsOrder, int indexInterative)
{
    // 根据实参传递过来的数值进行计算
    return nurbsOrder*(*(ctrlPointCor+indexInterative) - *(ctrlPointCor+indexInterative-1))/(*(knotVector+indexInterative+nurbsOrder) - *(knotVector+indexInterative));
}



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector(s),
 *    ssGetOutputPortSignal.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // 获取输入端口信号
    InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs ( S,0); 
    InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs ( S,1);
    
    // 获取输出端口信号和端口宽度
    real_T *ContourErrInfo = ssGetOutputPortRealSignal (S,0);
    real_T *TipContourError =  ssGetOutputPortRealSignal (S,1);
    real_T *OrienContourError =  ssGetOutputPortRealSignal (S,2);
    
    // 获取工作向量的信号值        
    real_T *ContourErrorXLastFirst  = (real_T*) ssGetDWork(S,0);
    real_T *ContourErrorXLastSecond = (real_T*) ssGetDWork(S,1);    
    real_T *ContourErrorXLastThird  = (real_T*) ssGetDWork(S,2);
    
    real_T *ContourErrorYLastFirst  = (real_T*) ssGetDWork(S,3);
    real_T *ContourErrorYLastSecond = (real_T*) ssGetDWork(S,4);    
    real_T *ContourErrorYLastThird  = (real_T*) ssGetDWork(S,5);

    real_T *ContourErrorZLastFirst  = (real_T*) ssGetDWork(S,6);
    real_T *ContourErrorZLastSecond = (real_T*) ssGetDWork(S,7);    
    real_T *ContourErrorZLastThird  = (real_T*) ssGetDWork(S,8);

    real_T *ContourErrorILastFirst  = (real_T*) ssGetDWork(S,9);
    real_T *ContourErrorILastSecond = (real_T*) ssGetDWork(S,10);    
    real_T *ContourErrorILastThird  = (real_T*) ssGetDWork(S,11);
    
    real_T *ContourErrorJLastFirst  = (real_T*) ssGetDWork(S,12);
    real_T *ContourErrorJLastSecond = (real_T*) ssGetDWork(S,13);    
    real_T *ContourErrorJLastThird  = (real_T*) ssGetDWork(S,14);    

    real_T *ContourErrorKLastFirst  = (real_T*) ssGetDWork(S,15);
    real_T *ContourErrorKLastSecond = (real_T*) ssGetDWork(S,16);    
    real_T *ContourErrorKLastThird  = (real_T*) ssGetDWork(S,17);
    
    real_T *FootPointCLast  = (real_T*) ssGetDWork(S,18);  
   
    // 获取仿真时间
    double t=ssGetT(S);
    
    // 下面的代码直接从C移植过来，用于计算轮廓误差
    // 定义用于循环计算时的临时索引变量
    int tempLoopIndex=0;
    // 定义临时变量，用于保存插补点坐标值
    double xKnotCor=0,yKnotCor=0,zKnotCor=0;
    // 定义计算得到的型值点的一阶导矢，即Cx'，Cy'，Cz'
    double xKnotCorDer1=0,yKnotCorDer1=0,zKnotCorDer1=0;
    // 定义计算得到的型值点的二阶导矢，即Cx"，Cy"，Cz"
    double xKnotCorDer2=0,yKnotCorDer2=0,zKnotCorDer2=0;
    // 定义计算得到的型值点的三阶导矢，即Cx"'，Cy"'，Cz"'
    double xKnotCorDer3=0,yKnotCorDer3=0,zKnotCorDer3=0;
    // 定义计算得到的刀轴方向的型值点
    double iKnotCor=0,jKnotCor=0,kKnotCor=0;
    // 定义最近插补点的曲率的分子和分母
    double nearestInterpCurvatureNum=0,nearestInterpCurvatureDen=0;
    // 定义最近插补点曲率
    double nearestInterpCurvature=0;
    // 定义法向矢量的两个坐标分量
    double normalVectorX=0,normalVectorY=0,normalVectorZ=0;
    // 定义跟踪误差
    double trackingErrX=0,trackingErrY=0,trackingErrZ=0;
    // 定义切向量分量，副法矢量分量，主法向量分量
    double tangentVectorX=0,tangentVectorY=0,tangentVectorZ=0,subNormalVectorX=0,subNormalVectorY=0,subNormalVectorZ=0;
    // 定义计算副法矢量过程中的一个分量
    double subNormalVectorX1=0,subNormalVectorY1=0,subNormalVectorZ1=0;
    // 定义一元三次方程中的系数
    double coefficienta=0, coefficientb=0, coefficientc=0, coefficientd=0;
    // 定义利用盛金公式解方程时，系数A/B/C
    double coefficientA=0, coefficientB=0, coefficientC=0, coefficientDelta=0,coefficientY1=0, coefficientY2=0, coefficientY1Root3=0, coefficientY2Root3=0, coefficientT=0, coefficientTheta=0;
    // 定义利用盛金公式求取的弧长增量
    double deltaS=0,deltaS1=0,deltaS2=0;
    // 定义临时变量，用于保存deBoor-Cox公式计算的型值点及各阶导失
    double deltaSxKnotCoor=0,deltaSyKnotCoor=0,deltaSzKnotCoor=0,deltaS1xKnotCoor=0,deltaS1yKnotCoor=0,deltaS1zKnotCoor=0,deltaS2xKnotCoor=0,deltaS2yKnotCoor=0,deltaS2zKnotCoor=0;
    double deltaSxKnotCoorDer1=0,deltaSyKnotCoorDer1=0,deltaSzKnotCoorDer1=0,deltaS1xKnotCoorDer1=0,deltaS1yKnotCoorDer1=0,deltaS1zKnotCoorDer1=0,deltaS2xKnotCoorDer1=0,deltaS2yKnotCoorDer1=0,deltaS2zKnotCoorDer1=0;
    double deltaSxKnotCoorDer2=0,deltaSyKnotCoorDer2=0,deltaSzKnotCoorDer2=0,deltaS1xKnotCoorDer2=0,deltaS1yKnotCoorDer2=0,deltaS1zKnotCoorDer2=0,deltaS2xKnotCoorDer2=0,deltaS2yKnotCoorDer2=0,deltaS2zKnotCoorDer2=0;
    double deltaSxKnotCoorDer3=0,deltaSyKnotCoorDer3=0,deltaSzKnotCoorDer3=0,deltaS1xKnotCoorDer3=0,deltaS1yKnotCoorDer3=0,deltaS1zKnotCoorDer3=0,deltaS2xKnotCoorDer3=0,deltaS2yKnotCoorDer3=0,deltaS2zKnotCoorDer3=0;
    double deltaSiKnotCoor=0,deltaSjKnotCoor=0,deltaSkKnotCoor=0,deltaS1iKnotCoor=0,deltaS1jKnotCoor=0,deltaS1kKnotCoor=0,deltaS2iKnotCoor=0,deltaS2jKnotCoor=0,deltaS2kKnotCoor=0;
    // 定义临时变量，用于保存取舍弧长过程中的型值点和各阶导失
    double tempxKnotCoor=0,tempyKnotCoor=0,tempzKnotCoor=0,tempxKnotCoorDer1=0,tempyKnotCoorDer1=0,tempzKnotCoorDer1=0,tempxKnotCoorDer2=0,tempyKnotCoorDer2=0,tempzKnotCoorDer2=0,tempxKnotCoorDer3=0,tempyKnotCoorDer3=0,tempzKnotCoorDer3=0;
    double tempiKnotCoor=0,tempjKnotCoor=0,tempkKnotCoor=0;
    // 定义临时变量，用于保存取舍弧长过程中的距离值
    double tempDistance=0;
    // 定义变量，保存最终足点型值点和各阶导失
    double footPointX=0, footPointY=0, footPointZ=0,footPointXDer1=0, footPointYDer1=0, footPointZDer1=0,footPointXDer2=0, footPointYDer2=0, footPointZDer2=0,footPointXDer3=0, footPointYDer3=0, footPointZDer3=0;
    double footPointI=0, footPointJ=0, footPointK=0;
    // 定义变量，保存足点处的单位方向向量
    double IdentityfootPointI=0,IdentityfootPointJ=0,IdentityfootPointK=0;
    // 定义临时变量，保存实际位置点与所求弧长对应的足点之间的距离
    double distanceFoot=0,distanceS=0,distanceS1=0,distanceS2=0;
    // 定义求足点的插补点的曲线参数
    double uNurbsfoot=0,uNurbsfoot1=0,uNurbsfoot2=0,tempuNurbs=0,uNurbsParaFoot=0;
    // 定义指针，用于保存查表过程中的指针变量
    real_T *pointerX,*pointerY,*pointerZ,*pointerI,*pointerJ,*pointerK,*pointerU;
    // 定义指针，用于保存接口的实际值
    real_T actualTipPosX=0,actualTipPosY=0,actualTipPosZ=0,actualOrienPosI=0,actualOrienPosJ=0,actualOrienPosK=0;
    // 定义指针，用于保存判断是否计算的条件
    int_T StartEnable=0;
    // 定义查表的迭代判断条件
    boolean_T  iterative=1;
    // 定义变量，保存最近插补点的曲线参数值
    double uNurbsnearest=0;
    // 定义变量，保存累计次数
    int numIterative=0;
    // 定义临时变量，保存计算过程中查表次数
    int numOfCalSearch=0;
    // 定义变量，保存最近插补点的曲线参数U值
    double uNurbsParaCurrent=0;
    // 定义变量，保存最近插补点上一插补点的曲线参数U值
    double uNurbsParaLast=0;    
    // 定义变量，保存最近插补点下一个插补点的曲线参数U值
    double uNurbsParaNext=0;
    // 定义变量，保存最近插补点的曲线规划的速度值
    double velProfileCurrent=0;
    // 定义变量，保存最近插补点上一插补点的曲线规划的速度值
    double velProfileLast=0;
    // 定义变量，保存最近插补点下一个插补点的曲线规划的速度值
    double velProfileNext=0;
    // 定义变量，保存最近插补点的曲线规划的加速度值
    double accProfileCurrent=0;
    // 定义变量，保存最近插补点上一插补点的曲线规划的加速度值
    double accProfileLast=0;
    // 定义变量，保存最近插补点下一个插补点的曲线规划的加速度值
    double accProfileNext=0; 
    // 定义临时指针，用于保存调用DeboorToolTipOrien函数返回的数组地址
    double *pointerDeboorRT;
    // 定义变量，保存跟踪误差在法向上的分量
    double normalComponent=0;
    // 定义变量，保存跟踪误差在副法向上的分量
    double binormalComponent=0;
    // 定义变量，保存足点处对应的旋转轴的角度值
    double FootPointRotA=0,FootPointRotC=0;
    // 定义变量，保存旋转轴实际的角度值
    double actualRotA=0,actualRotC=0;
    // 定义变量，保存实际轮廓误差分量值
    double ContourErrComponentX=0,ContourErrComponentY=0,ContourErrComponentZ=0,ContourErrComponentI=0,ContourErrComponentJ=0,ContourErrComponentK=0;
    // 定义变量，保存实际轮廓误差分量值的一阶导失
    double ContourErrComponentXDer1=0,ContourErrComponentYDer1=0,ContourErrComponentZDer1=0,ContourErrComponentIDer1=0,ContourErrComponentJDer1=0,ContourErrComponentKDer1=0;
    // 定义变量，保存实际轮廓误差分量值的二阶导失
    double ContourErrComponentXDer2=0,ContourErrComponentYDer2=0,ContourErrComponentZDer2=0,ContourErrComponentIDer2=0,ContourErrComponentJDer2=0,ContourErrComponentKDer2=0;
    // 定义变量，用于保存计算方向轮廓误差的比例关系
    double Ratio=0;
    // 定义变量，用于保存最近插补点处和其上一时刻或下一时刻的刀轴方向信息
    double refPointValueINearest=0,refPointValueJNearest=0,refPointValueKNearest=0;
    double refPointValueINearestLast=0,refPointValueJNearestLast=0,refPointValueKNearestLast=0;
    double refPointValueINearestNext=0,refPointValueJNearestNext=0,refPointValueKNearestNext=0;
    // 定义变量，用于保存足点处的方向向量
    double ToolOrientationI=0,ToolOrientationJ=0,ToolOrientationK=0;
    // 定义变量，用于保存足点处的方向向量的单位化值
    double ToolOrientationIentityI=0,ToolOrientationIentityJ=0,ToolOrientationIentityK=0;
    
    // 赋值索引，用于循环仿真
    numOfCalSearch=numOfCal;

    // 保存查表过程中的指针向量
    // 从端口中获取数据
    StartEnable=*uPtrs0[0];   
    actualTipPosX=*uPtrs1[0];
    actualTipPosY=*uPtrs1[1];
    actualTipPosZ=*uPtrs1[2];
    actualOrienPosI=*uPtrs1[3];
    actualOrienPosJ=*uPtrs1[4];
    actualOrienPosK=*uPtrs1[5];
    actualRotA=*uPtrs1[6];
    actualRotC=*uPtrs1[7];
    pointerX=refPointValueX;
    pointerY=refPointValueY;
    pointerZ=refPointValueZ; 
    

    if (StartEnable==1)
    {               
        switch (numOfCal)
        {
        case 1: case 2: case 3: numIterative=1;  break;
        default: while (iterative && numOfCalSearch)
                 {
                     iterative=(pow((actualTipPosX-(*pointerX)),2)+pow((actualTipPosY-(*pointerY)),2)+pow((actualTipPosZ-(*pointerZ)),2))>
                         (pow((actualTipPosX-(*(pointerX-1))),2)+pow((actualTipPosY-(*(pointerY-1))),2)+pow((actualTipPosZ-(*(pointerZ-1))),2));
                     pointerX--;
                     pointerY--;
                     pointerZ--;
                     numIterative++;
                     numOfCalSearch--;
                 }
        }
        // 定义临时指针，用于保存最近插补点的指针信息 
        pointerU=uNurbs-numIterative+1;
        pointerI=refPointValueI-numIterative+1;
        pointerJ=refPointValueJ-numIterative+1;
        pointerK=refPointValueK-numIterative+1;

        // 读取向后扫描得到的离实际位置点最近的插补点的相应信息
        uNurbsnearest=*pointerU;
        refPointValueINearest=*pointerI;
        refPointValueJNearest=*pointerJ;        
        refPointValueKNearest=*pointerK;
        
        // 读取向后扫描获得的离实际位置点最近的插补点的上一时刻或者下一时刻的刀轴方向信息
        refPointValueINearestLast=*(pointerI-1);
        refPointValueJNearestLast=*(pointerJ-1);
        refPointValueKNearestLast=*(pointerK-1);
        refPointValueINearestNext=*(pointerI+1);
        refPointValueJNearestNext=*(pointerJ+1);
        refPointValueKNearestNext=*(pointerK+1); 

        // 计算deboor型值点和各阶导失
        pointerDeboorRT=DeboorToolTipOrien(uNurbsnearest);
        xKnotCor=*pointerDeboorRT;
        yKnotCor=*(pointerDeboorRT+1);
        zKnotCor=*(pointerDeboorRT+2);
        xKnotCorDer1=*(pointerDeboorRT+3);
        yKnotCorDer1=*(pointerDeboorRT+4);
        zKnotCorDer1=*(pointerDeboorRT+5);
        xKnotCorDer2=*(pointerDeboorRT+6);
        yKnotCorDer2=*(pointerDeboorRT+7);
        zKnotCorDer2=*(pointerDeboorRT+8);
        iKnotCor=*(pointerDeboorRT+12);
        jKnotCor=*(pointerDeboorRT+13);        
        kKnotCor=*(pointerDeboorRT+14);        

        // 采用盛金公式计算与实际位置点最近的插补点与足点之间的弧长增量deltaS
        // 首先计算切矢量
        tangentVectorX=xKnotCorDer1/sqrt(pow(xKnotCorDer1,2)+pow(yKnotCorDer1,2)+pow(zKnotCorDer1,2));
        tangentVectorY=yKnotCorDer1/sqrt(pow(xKnotCorDer1,2)+pow(yKnotCorDer1,2)+pow(zKnotCorDer1,2));
        tangentVectorZ=zKnotCorDer1/sqrt(pow(xKnotCorDer1,2)+pow(yKnotCorDer1,2)+pow(zKnotCorDer1,2));

        // 再计算副法矢量的分量
        subNormalVectorX1=yKnotCorDer1*zKnotCorDer2-yKnotCorDer2*zKnotCorDer1;
        subNormalVectorY1=-xKnotCorDer1*zKnotCorDer2+xKnotCorDer2*zKnotCorDer1;
        subNormalVectorZ1=xKnotCorDer1*yKnotCorDer2-xKnotCorDer2*yKnotCorDer1;

        // 再计算副法矢量
        subNormalVectorX=subNormalVectorX1/sqrt(pow(subNormalVectorX1,2)+pow(subNormalVectorY1,2)+pow(subNormalVectorZ1,2));
        subNormalVectorY=subNormalVectorY1/sqrt(pow(subNormalVectorX1,2)+pow(subNormalVectorY1,2)+pow(subNormalVectorZ1,2));
        subNormalVectorZ=subNormalVectorZ1/sqrt(pow(subNormalVectorX1,2)+pow(subNormalVectorY1,2)+pow(subNormalVectorZ1,2));

        // 计算主法向量
        normalVectorX=subNormalVectorY*tangentVectorZ-subNormalVectorZ*tangentVectorY;
        normalVectorY=subNormalVectorZ*tangentVectorX-subNormalVectorX*tangentVectorZ;
        normalVectorZ=subNormalVectorX*tangentVectorY-subNormalVectorY*tangentVectorX;

        // 计算最近插补点的曲率
        nearestInterpCurvatureNum=sqrt(pow(subNormalVectorX1,2)+pow(subNormalVectorY1,2)+pow(subNormalVectorZ1,2));
        nearestInterpCurvatureDen=pow(sqrt(pow(xKnotCorDer1,2)+pow(yKnotCorDer1,2)+pow(zKnotCorDer1,2)),3.0);
        nearestInterpCurvature=nearestInterpCurvatureNum/nearestInterpCurvatureDen;

        // 计算一元三次方程的系数
        coefficienta=0.5*pow(nearestInterpCurvature,2);
        coefficientb=0;
        coefficientc=1+nearestInterpCurvature*(xKnotCor*normalVectorX+yKnotCor*normalVectorY+zKnotCor*normalVectorZ)
            -nearestInterpCurvature*(actualTipPosX*normalVectorX+actualTipPosY*normalVectorY+actualTipPosZ*normalVectorZ);
        coefficientd=(xKnotCor-actualTipPosX)*tangentVectorX+(yKnotCor-actualTipPosY)*tangentVectorY+(zKnotCor-actualTipPosZ)*tangentVectorZ;

        // 计算盛金公式中的重根判别式系数
        coefficientA=coefficientb*coefficientb-3*coefficienta*coefficientc;
        coefficientB=coefficientb*coefficientc-9*coefficienta*coefficientd;
        coefficientC=coefficientc*coefficientc-3*coefficientb*coefficientd;
        coefficientDelta=coefficientB*coefficientB-4*coefficientA*coefficientC;

        // 判断A=B=0是否成立，若成立则有三重根
        if ((coefficientA==coefficientB) && (coefficientB==0))
        {
            deltaS=-coefficientb/(3*coefficienta);
            deltaS1=-coefficientb/(3*coefficienta);
            deltaS2=-coefficientb/(3*coefficienta);
        } 
        // 判断总判别式是否>0，是则有一实根，一对共轭虚根
        else if (coefficientDelta>0)
        {
            coefficientY1=coefficientA*coefficientb+3*coefficienta*(-coefficientB+sqrt(coefficientB*coefficientB-4*coefficientA*coefficientC))/2.0;
            coefficientY2=coefficientA*coefficientb+3*coefficienta*(-coefficientB-sqrt(coefficientB*coefficientB-4*coefficientA*coefficientC))/2.0;
            if (coefficientY1>=0)
            {
                coefficientY1Root3=pow(coefficientY1,1.0/3.0);
            }
            else
            {
                coefficientY1Root3=-pow(-coefficientY1,1.0/3.0);
            }
            if (coefficientY2>=0)
            {
                coefficientY2Root3=pow(coefficientY2,1.0/3.0);
            }
            else
            {
                coefficientY2Root3=-pow(-coefficientY2,1.0/3.0);
            }
            deltaS=(-coefficientb-coefficientY1Root3-coefficientY2Root3)/(3*coefficienta);
            deltaS1=(-coefficientb-coefficientY1Root3-coefficientY2Root3)/(3*coefficienta);
            deltaS2=(-coefficientb-coefficientY1Root3-coefficientY2Root3)/(3*coefficienta);
        }
        // 判断总判别式是否=0，是则有三实根，其中有两重根
        else if (coefficientDelta==0)
        {
            deltaS=-coefficientb/coefficienta+coefficientB/coefficientA;
            deltaS1=-coefficientB/(2*coefficientA);
            deltaS2=-coefficientB/(2*coefficientA);
            } 
        // 判断总判别式是否<0，是则有三不等实根
        else
        {
            if (coefficientA>=0)
            {
                coefficientT=(2*coefficientA*coefficientb-3*coefficienta*coefficientB)/(2*pow(coefficientA,3.0/2.0));
            } 
            else
            {
                coefficientT=(2*coefficientA*coefficientb-3*coefficienta*coefficientB)/(-2*pow(-coefficientA,3.0/2.0));
            }
            coefficientTheta=acos(coefficientT);
            if (coefficientA<0)
            {
                printf("\n");
            }
            else
            {
                deltaS=(-coefficientb-2*sqrt(coefficientA)*cos(coefficientTheta/3.0))/(3*coefficienta);
                deltaS1=(-coefficientb+sqrt(coefficientA)*(cos(coefficientTheta/3.0)+sqrt(3.0)*sin(coefficientTheta/3.0)))/(3*coefficienta);
                deltaS2=(-coefficientb+sqrt(coefficientA)*(cos(coefficientTheta/3.0)-sqrt(3.0)*sin(coefficientTheta/3.0)))/(3*coefficienta);
            }
        }

        // 根据盛金公式求取的弧长增量计算足点的曲线参数
        uNurbsfoot=uNurbsnearest+1.0/sqrt(pow(xKnotCorDer1,2)+pow(yKnotCorDer1,2)+pow(zKnotCorDer1,2))*deltaS
            -0.5*(xKnotCorDer1*xKnotCorDer2+yKnotCorDer1*yKnotCorDer2+zKnotCorDer1*zKnotCorDer2)/pow(sqrt(pow(xKnotCorDer1,2)+pow(yKnotCorDer1,2)+pow(zKnotCorDer1,2)),4.0)*deltaS*deltaS;
        uNurbsfoot1=uNurbsnearest+1.0/sqrt(pow(xKnotCorDer1,2)+pow(yKnotCorDer1,2)+pow(zKnotCorDer1,2))*deltaS1
            -0.5*(xKnotCorDer1*xKnotCorDer2+yKnotCorDer1*yKnotCorDer2+zKnotCorDer1*zKnotCorDer2)/pow(sqrt(pow(xKnotCorDer1,2)+pow(yKnotCorDer1,2)+pow(zKnotCorDer1,2)),4.0)*deltaS1*deltaS1;
        uNurbsfoot2=uNurbsnearest+1.0/sqrt(pow(xKnotCorDer1,2)+pow(yKnotCorDer1,2)+pow(zKnotCorDer1,2))*deltaS2
            -0.5*(xKnotCorDer1*xKnotCorDer2+yKnotCorDer1*yKnotCorDer2+zKnotCorDer1*zKnotCorDer2)/pow(sqrt(pow(xKnotCorDer1,2)+pow(yKnotCorDer1,2)+pow(zKnotCorDer1,2)),4.0)*deltaS2*deltaS2;

        //利用deBoor-Cox公式求足点对应的型值点
        if (uNurbsfoot<0)
        {
            uNurbsfoot=0;
        }
        else if (uNurbsfoot>1)
        {
            uNurbsfoot=1;
        }
        pointerDeboorRT=DeboorToolTipOrien(uNurbsfoot);
        deltaSxKnotCoor=*pointerDeboorRT;
        deltaSyKnotCoor=*(pointerDeboorRT+1);
        deltaSzKnotCoor=*(pointerDeboorRT+2);
        deltaSxKnotCoorDer1=*(pointerDeboorRT+3);
        deltaSyKnotCoorDer1=*(pointerDeboorRT+4);
        deltaSzKnotCoorDer1=*(pointerDeboorRT+5);
        deltaSxKnotCoorDer2=*(pointerDeboorRT+6);
        deltaSyKnotCoorDer2=*(pointerDeboorRT+7);
        deltaSzKnotCoorDer2=*(pointerDeboorRT+8);
        deltaSxKnotCoorDer3=*(pointerDeboorRT+9);
        deltaSyKnotCoorDer3=*(pointerDeboorRT+10);
        deltaSzKnotCoorDer3=*(pointerDeboorRT+11);
        deltaSiKnotCoor=*(pointerDeboorRT+12);
        deltaSjKnotCoor=*(pointerDeboorRT+13);
        deltaSkKnotCoor=*(pointerDeboorRT+14);    

        if (uNurbsfoot1<0)
        {
            uNurbsfoot1=0;
        }
        else if (uNurbsfoot1>1)
        {
            uNurbsfoot1=1;
        }
        pointerDeboorRT=DeboorToolTipOrien(uNurbsfoot1);
        deltaS1xKnotCoor=*pointerDeboorRT;
        deltaS1yKnotCoor=*(pointerDeboorRT+1);
        deltaS1zKnotCoor=*(pointerDeboorRT+2);
        deltaS1xKnotCoorDer1=*(pointerDeboorRT+3);
        deltaS1yKnotCoorDer1=*(pointerDeboorRT+4);
        deltaS1zKnotCoorDer1=*(pointerDeboorRT+5);
        deltaS1xKnotCoorDer2=*(pointerDeboorRT+6);
        deltaS1yKnotCoorDer2=*(pointerDeboorRT+7);
        deltaS1zKnotCoorDer2=*(pointerDeboorRT+8);
        deltaS1xKnotCoorDer3=*(pointerDeboorRT+9);
        deltaS1yKnotCoorDer3=*(pointerDeboorRT+10);
        deltaS1zKnotCoorDer3=*(pointerDeboorRT+11);
        deltaS1iKnotCoor=*(pointerDeboorRT+12);
        deltaS1jKnotCoor=*(pointerDeboorRT+13);
        deltaS1kKnotCoor=*(pointerDeboorRT+14);       

        if (uNurbsfoot2<0)
        {
            uNurbsfoot2=0;
        }
        else if (uNurbsfoot2>1)
        {
            uNurbsfoot2=1;
        }
        pointerDeboorRT=DeboorToolTipOrien(uNurbsfoot2);
        deltaS2xKnotCoor=*pointerDeboorRT;
        deltaS2yKnotCoor=*(pointerDeboorRT+1);
        deltaS2zKnotCoor=*(pointerDeboorRT+2);
        deltaS2xKnotCoorDer1=*(pointerDeboorRT+3);
        deltaS2yKnotCoorDer1=*(pointerDeboorRT+4);
        deltaS2zKnotCoorDer1=*(pointerDeboorRT+5);
        deltaS2xKnotCoorDer2=*(pointerDeboorRT+6);
        deltaS2yKnotCoorDer2=*(pointerDeboorRT+7);
        deltaS2zKnotCoorDer2=*(pointerDeboorRT+8);
        deltaS2xKnotCoorDer3=*(pointerDeboorRT+9);
        deltaS2yKnotCoorDer3=*(pointerDeboorRT+10);
        deltaS2zKnotCoorDer3=*(pointerDeboorRT+11);
        deltaS2iKnotCoor=*(pointerDeboorRT+12);
        deltaS2jKnotCoor=*(pointerDeboorRT+13);
        deltaS2kKnotCoor=*(pointerDeboorRT+14);

        //根据实际位置点与足点最短距离的要求，确定足点
        distanceS=pow((actualTipPosX-deltaSxKnotCoor),2)+pow((actualTipPosY-deltaSyKnotCoor),2)+pow((actualTipPosZ-deltaSzKnotCoor),2);
        distanceS1=pow((actualTipPosX-deltaS1xKnotCoor),2)+pow((actualTipPosY-deltaS1yKnotCoor),2)+pow((actualTipPosZ-deltaS1zKnotCoor),2);
        distanceS2=pow((actualTipPosX-deltaS2xKnotCoor),2)+pow((actualTipPosY-deltaS2yKnotCoor),2)+pow((actualTipPosZ-deltaS2zKnotCoor),2);
        if (distanceS>=distanceS1)
        {
            tempDistance=distanceS1;
            tempxKnotCoor=deltaS1xKnotCoor;
            tempyKnotCoor=deltaS1yKnotCoor;
            tempzKnotCoor=deltaS1zKnotCoor;
            tempxKnotCoorDer1=deltaS1xKnotCoorDer1;
            tempyKnotCoorDer1=deltaS1yKnotCoorDer1;
            tempzKnotCoorDer1=deltaS1zKnotCoorDer1;
            tempxKnotCoorDer2=deltaS1xKnotCoorDer2;
            tempyKnotCoorDer2=deltaS1yKnotCoorDer2;
            tempzKnotCoorDer2=deltaS1zKnotCoorDer2;
            tempxKnotCoorDer3=deltaS1xKnotCoorDer3;
            tempyKnotCoorDer3=deltaS1yKnotCoorDer3;
            tempzKnotCoorDer3=deltaS1zKnotCoorDer3;
            tempiKnotCoor=deltaSiKnotCoor;
            tempjKnotCoor=deltaSjKnotCoor;
            tempkKnotCoor=deltaSkKnotCoor;            
            tempuNurbs=uNurbsfoot1;
        }
        else
        {
            tempDistance=distanceS;
            tempxKnotCoor=deltaSxKnotCoor;
            tempyKnotCoor=deltaSyKnotCoor;
            tempzKnotCoor=deltaSzKnotCoor;
            tempxKnotCoorDer1=deltaSxKnotCoorDer1;
            tempyKnotCoorDer1=deltaSyKnotCoorDer1;
            tempzKnotCoorDer1=deltaSzKnotCoorDer1;
            tempxKnotCoorDer2=deltaSxKnotCoorDer2;
            tempyKnotCoorDer2=deltaSyKnotCoorDer2;
            tempzKnotCoorDer2=deltaSzKnotCoorDer2;
            tempxKnotCoorDer3=deltaSxKnotCoorDer3;
            tempyKnotCoorDer3=deltaSyKnotCoorDer3;
            tempzKnotCoorDer3=deltaSzKnotCoorDer3;
            tempiKnotCoor=deltaS1iKnotCoor;
            tempjKnotCoor=deltaS1jKnotCoor;
            tempkKnotCoor=deltaS1kKnotCoor;
            tempuNurbs=uNurbsfoot;
        }
        if (tempDistance>=distanceS2)
        {
            footPointX=deltaS2xKnotCoor;
            footPointY=deltaS2yKnotCoor;
            footPointZ=deltaS2zKnotCoor;
            footPointXDer1=deltaS2xKnotCoorDer1;
            footPointYDer1=deltaS2yKnotCoorDer1;
            footPointZDer1=deltaS2zKnotCoorDer1;
            footPointXDer2=deltaS2xKnotCoorDer2;
            footPointYDer2=deltaS2yKnotCoorDer2;
            footPointZDer2=deltaS2zKnotCoorDer2;
            footPointXDer3=deltaS2xKnotCoorDer3;
            footPointYDer3=deltaS2yKnotCoorDer3;
            footPointZDer3=deltaS2zKnotCoorDer3;
            footPointI=deltaS2iKnotCoor;
            footPointJ=deltaS2jKnotCoor;
            footPointK=deltaS2kKnotCoor;
            uNurbsParaFoot=uNurbsfoot2;
        } 
        else
        {
            footPointX=tempxKnotCoor;
            footPointY=tempyKnotCoor;
            footPointZ=tempzKnotCoor;
            footPointXDer1=tempxKnotCoorDer1;
            footPointYDer1=tempyKnotCoorDer1;
            footPointZDer1=tempzKnotCoorDer1;
            footPointXDer2=tempxKnotCoorDer2;
            footPointYDer2=tempyKnotCoorDer2;
            footPointZDer2=tempzKnotCoorDer2;
            footPointXDer3=tempxKnotCoorDer3;
            footPointYDer3=tempyKnotCoorDer3;
            footPointZDer3=tempzKnotCoorDer3;
            footPointI=tempiKnotCoor;
            footPointJ=tempjKnotCoor;            
            footPointK=tempkKnotCoor;            
            uNurbsParaFoot=tempuNurbs;
        }
        
        // 将获得的足点处的方向向量单位化
        IdentityfootPointI=footPointI/sqrt(pow(footPointI,2)+pow(footPointJ,2)+pow(footPointK,2));
        IdentityfootPointJ=footPointJ/sqrt(pow(footPointI,2)+pow(footPointJ,2)+pow(footPointK,2));
        IdentityfootPointK=footPointK/sqrt(pow(footPointI,2)+pow(footPointJ,2)+pow(footPointK,2));        
        
        // 计算刀轴方向轮廓误差
        OrienContourError[0]=acos((IdentityfootPointI*actualOrienPosI+IdentityfootPointJ*actualOrienPosJ+IdentityfootPointK*actualOrienPosK)/
        (sqrt(pow(IdentityfootPointI,2)+pow(IdentityfootPointJ,2)+pow(IdentityfootPointK,2))*sqrt(pow(actualOrienPosI,2)+pow(actualOrienPosJ,2)+pow(actualOrienPosK,2))));
        
        // 计算足点处对应的旋转轴的角度值
        FootPointRotA=acos(IdentityfootPointK);
        FootPointRotC=atan2(IdentityfootPointI,IdentityfootPointJ);
        
        if ((abs(FootPointRotC-FootPointCLast[0]))>=6)
        {
            FootPointRotC=FootPointRotC-2*pi;
        }
                
        // 利用足点计算轮廓误差分量
        ContourErrComponentX=footPointX-actualTipPosX;
        ContourErrComponentY=footPointY-actualTipPosY;
        ContourErrComponentZ=footPointZ-actualTipPosZ;
        ContourErrComponentI=IdentityfootPointI-actualOrienPosI;
        ContourErrComponentJ=IdentityfootPointJ-actualOrienPosJ;
        ContourErrComponentK=IdentityfootPointK-actualOrienPosK;        

        // 计算刀尖点轮廓误差
        TipContourError[0]=sqrt(pow(footPointX-actualTipPosX,2)+pow(footPointY-actualTipPosY,2)+pow(footPointZ-actualTipPosZ,2));
        
        // 保存轮廓点信息
        ContourErrInfo[0]=ContourErrComponentX;
        ContourErrInfo[1]=ContourErrComponentY;
        ContourErrInfo[2]=ContourErrComponentZ;
        ContourErrInfo[3]=ContourErrComponentI;
        ContourErrInfo[4]=ContourErrComponentJ;
        ContourErrInfo[5]=ContourErrComponentK;        
        
        // 计算轮廓误差分量的一阶导失
        ContourErrComponentXDer1=(3*ContourErrInfo[0]-4*ContourErrorXLastFirst[0]+ContourErrorXLastSecond[0])/(2*SimStep);
        ContourErrComponentYDer1=(3*ContourErrInfo[1]-4*ContourErrorYLastFirst[0]+ContourErrorYLastSecond[0])/(2*SimStep);
        ContourErrComponentZDer1=(3*ContourErrInfo[2]-4*ContourErrorZLastFirst[0]+ContourErrorZLastSecond[0])/(2*SimStep);
        ContourErrComponentIDer1=(3*ContourErrInfo[3]-4*ContourErrorILastFirst[0]+ContourErrorILastSecond[0])/(2*SimStep);
        ContourErrComponentJDer1=(3*ContourErrInfo[4]-4*ContourErrorJLastFirst[0]+ContourErrorJLastSecond[0])/(2*SimStep);
        ContourErrComponentKDer1=(3*ContourErrInfo[5]-4*ContourErrorKLastFirst[0]+ContourErrorKLastSecond[0])/(2*SimStep);        
                
        // 保存轮廓误差分量的一阶导信息
        ContourErrInfo[6]=ContourErrComponentXDer1;
        ContourErrInfo[7]=ContourErrComponentYDer1;
        ContourErrInfo[8]=ContourErrComponentZDer1;
        ContourErrInfo[9]=ContourErrComponentIDer1;
        ContourErrInfo[10]=ContourErrComponentJDer1;
        ContourErrInfo[11]=ContourErrComponentKDer1;
          
        // 计算轮廓误差分量的二阶导失
        ContourErrComponentXDer2=(2*ContourErrInfo[0]-5*ContourErrorXLastFirst[0]+4*ContourErrorXLastSecond[0]-ContourErrorXLastThird[0])/(SimStep*SimStep);
        ContourErrComponentYDer2=(2*ContourErrInfo[1]-5*ContourErrorYLastFirst[0]+4*ContourErrorYLastSecond[0]-ContourErrorYLastThird[0])/(SimStep*SimStep);
        ContourErrComponentZDer2=(2*ContourErrInfo[2]-5*ContourErrorZLastFirst[0]+4*ContourErrorZLastSecond[0]-ContourErrorZLastThird[0])/(SimStep*SimStep);
        ContourErrComponentIDer2=(2*ContourErrInfo[3]-5*ContourErrorILastFirst[0]+4*ContourErrorILastSecond[0]-ContourErrorILastThird[0])/(SimStep*SimStep);
        ContourErrComponentJDer2=(2*ContourErrInfo[4]-5*ContourErrorJLastFirst[0]+4*ContourErrorJLastSecond[0]-ContourErrorJLastThird[0])/(SimStep*SimStep);
        ContourErrComponentKDer2=(2*ContourErrInfo[5]-5*ContourErrorKLastFirst[0]+4*ContourErrorKLastSecond[0]-ContourErrorKLastThird[0])/(SimStep*SimStep);        
                
        // 保存轮廓误差分量的一阶导信息        
        ContourErrInfo[12]=ContourErrComponentXDer2;
        ContourErrInfo[13]=ContourErrComponentYDer2;
        ContourErrInfo[14]=ContourErrComponentZDer2;
        ContourErrInfo[15]=ContourErrComponentIDer2;
        ContourErrInfo[16]=ContourErrComponentJDer2;
        ContourErrInfo[17]=ContourErrComponentKDer2;        
                
        // 保存刀轴方向的轮廓误差分量信息
        ContourErrInfo[18]=FootPointRotC;

        // 使指针自动加一，以指向下一个参数
        refPointValueX++;
        refPointValueY++;
        refPointValueZ++;
		refPointValueI++;
		refPointValueJ++;
		refPointValueK++;
        uNurbs++;    
        numOfCal++;
    }/* end if(ControlEnable==1) */
}/* end mdlOutputs */


#define MDL_UPDATE
/* Function: mdlUpdate =========================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    // 获取输出端口的信号值和工作向量的信号值
    real_T  *ContourErrInfo=ssGetOutputPortRealSignal (S,0);
        
    real_T *ContourErrorXLastFirst  = (real_T*) ssGetDWork(S,0);
    real_T *ContourErrorXLastSecond = (real_T*) ssGetDWork(S,1);    
    real_T *ContourErrorXLastThird  = (real_T*) ssGetDWork(S,2);
    
    real_T *ContourErrorYLastFirst  = (real_T*) ssGetDWork(S,3);
    real_T *ContourErrorYLastSecond = (real_T*) ssGetDWork(S,4);    
    real_T *ContourErrorYLastThird  = (real_T*) ssGetDWork(S,5);

    real_T *ContourErrorZLastFirst  = (real_T*) ssGetDWork(S,6);
    real_T *ContourErrorZLastSecond = (real_T*) ssGetDWork(S,7);    
    real_T *ContourErrorZLastThird  = (real_T*) ssGetDWork(S,8);

    real_T *ContourErrorILastFirst  = (real_T*) ssGetDWork(S,9);
    real_T *ContourErrorILastSecond = (real_T*) ssGetDWork(S,10);    
    real_T *ContourErrorILastThird  = (real_T*) ssGetDWork(S,11);
    
    real_T *ContourErrorJLastFirst  = (real_T*) ssGetDWork(S,12);
    real_T *ContourErrorJLastSecond = (real_T*) ssGetDWork(S,13);    
    real_T *ContourErrorJLastThird  = (real_T*) ssGetDWork(S,14);    

    real_T *ContourErrorKLastFirst  = (real_T*) ssGetDWork(S,15);
    real_T *ContourErrorKLastSecond = (real_T*) ssGetDWork(S,16);    
    real_T *ContourErrorKLastThird  = (real_T*) ssGetDWork(S,17);
    
    real_T *FootPointCLast  = (real_T*) ssGetDWork(S,18); 

	FootPointCLast[0]=ContourErrInfo[18];

    // 将输出端口的信号值分别赋给十八个工作向量
    ContourErrorXLastThird[0] =ContourErrorXLastSecond[0];
    ContourErrorXLastSecond[0]=ContourErrorXLastFirst[0];
    ContourErrorXLastFirst[0] =ContourErrInfo[0];
    
    ContourErrorYLastThird[0] =ContourErrorYLastSecond[0];
    ContourErrorYLastSecond[0]=ContourErrorYLastFirst[0];
    ContourErrorYLastFirst[0] =ContourErrInfo[1];
    
    ContourErrorZLastThird[0] =ContourErrorZLastSecond[0];
    ContourErrorZLastSecond[0]=ContourErrorZLastFirst[0];
    ContourErrorZLastFirst[0] =ContourErrInfo[2];
    
    ContourErrorILastThird[0] =ContourErrorILastSecond[0];
    ContourErrorILastSecond[0]=ContourErrorILastFirst[0];
    ContourErrorILastFirst[0] =ContourErrInfo[3];
    
    ContourErrorJLastThird[0] =ContourErrorJLastSecond[0];
    ContourErrorJLastSecond[0]=ContourErrorJLastFirst[0];
    ContourErrorJLastFirst[0] =ContourErrInfo[4];
    
    ContourErrorKLastThird[0] =ContourErrorKLastSecond[0];
    ContourErrorKLastSecond[0]=ContourErrorKLastFirst[0];
    ContourErrorKLastFirst[0] =ContourErrInfo[5];
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.
*/
static void mdlTerminate(SimStruct *S)
{ 
} /* end mdlTerminate */  


/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

