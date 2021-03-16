/**************************************************************************************************************                             
**                                天津启诚伟业科技有限公司
**                                     
**                                 http://www.qcmcu.com
**                                 
**                                 Modified by：Chen likao
**************************************************************************************************************/

#include "Mouse_Drive.h"
#include "math.h"
#include <mouseclub.h>
/*********************************************************************************************************
  定义全局变量
*********************************************************************************************************/
MAZECOOR          GmcMouse                        = {0,0};              /*  保存电脑鼠当前位置坐标      */
uint8             GucMouseDir                     = UP;                 /*  保存电脑鼠当前方向          */
uint8             GucMapBlock[MAZETYPE][MAZETYPE] = {0};                /*  GucMapBlock[x][y]           */
                                                                           /*  x,横坐标;y,纵坐标;          */


uint8             GucMapBlock0[MAZETYPE][MAZETYPE] = {0};               /*  中心算法         */
                                                                   /*  bit3~bit0分别代表左下右上   */
                                                                        /*  0:该方向无路，1:该方向有路  */
uint8             GucMapBlock1[MAZETYPE][MAZETYPE]= {0x0f};
uint8             GucMouseStart                    = 0;                 /* 电脑鼠启动        */
uint8             GucFrontJinju                    = 0;                 /* 前方红外近距在搜索时等高图制作中用   */
uint8             GucCrossroad                     = 0;                 /* 十字路口数，冲刺时用，若十字路口多降低最高冲刺速度   */
static uint32     GW;                                                       /*小车转动角度*/
static float piancha_r;
static __MOTOR  __GmLeft                          = {0, 0, 0, 0, 0, 0};    /*  定义并初始化左电机状态      */
static __MOTOR  __GmRight                         = {0, 0, 0, 0, 0, 0};    /*  定义并初始化右电机状态      */
static __PID    __GmLPID;                                                 /*  定义左电机PID      */
static __PID    __GmRPID;                                                 /*  定义右电机PID     */
 __PID    __GmSPID;                                                 /*  直线PID     */
 __PID    __GmWPID;                                                 /*  旋转PID     */
static uint8    __GucMouseState                   = __STOP;             /*  保存电脑鼠当前运行状态      */
static int32    __GiMaxSpeed                      = SEARCHSPEED;        /*  保存允许运行的最大速度      */
static int32    __GiMinSpeed                      = SEARCHSPEED;
static uint8    __GucDistance[5]                  = {0};                /*  记录传感器状态              */
uint16   GusFreq_F                         = 36200;   //33.8,33,327        /*  前方红外频率              */
uint16   GusFreq_FJ                        = 19200;   //26.3,266,275              /*  前方近距红外频率              */
uint16   GusFreq_X                         = 30000;   //35,33.8          /*  斜45度红外频率              */
uint16   GusFreq_LF                        = 31700;   //34000           /*  左右红外远距频率              */
uint16   GusFreq_L                         = 18300;              /*  左右红外近距频率              */
static  float   GsTpusle_T                       = 0;    //int16              /*  左轮校正减少的速度值              */
static uint8    GuiSpeedCtr                       = 0;
static uint16   GuiTpusle_LR                      = 0;
static uint16   GuiTpusle_S                       = 0;
static uint8    GucFrontNear                      = 0;
static float jiao = 0;
uint8      map                             = 0;
uint8    GucGoHead                     = 0;
  uint8    GucGoHead1                     = 0;
   uint8    GucGoHead2                     = 0;
uint16    GucFangXiang                      = 0;
uint8    GucDirTemp                        = 0;
uint16    DIS[10] = {0};
uint8 Tab=4;//红外调试选项
uint8  Conturns                               =0;    
uint16 count=0;
uint8      m                             = 0;
uint8      backflag                        =0;
uint8 GoHead45_flag=0;
extern uint8 turn45_flag;
extern u16 voltageDetectRef;
extern __IO uint16_t ADC_ConvertedValue[5];
extern float ADC_ConvertedValueLocal[4]; 
float W;
uint16 Sunlight[4];
uint16 ucIRCheck[4];
extern uint8 usepiancha;
static float W_l;                       
static float W_r;
u32 Angle_TLY[8] = {0}; 
u32 Angle_TLY_All = 0;
u32 Angle_TLY_Average = 0;
u16 voltageDetectRef=2041;
float W;
float W_R;
 
static uint8 after_turnleft;
static float piancha_l;
 
//extern uint8 usepiancha=0;
float piancha_vl;
float piancha_vr;

/**红外拟合曲线数组**/
float disCheck3[201]={227.358,218.727,210.974,203.961,197.577,191.735,186.363,181.401,176.8,172.518,168.521,164.779,161.266,157.96,154.84,151.891,149.098,146.447,143.928,141.528,139.24,137.055,134.965,132.965,131.047,129.206,127.438,125.737,124.1,122.523,121.001,119.533,118.115,116.744,115.417,114.133,112.889,111.684,110.514,109.379,108.277,107.206,106.165,105.152,104.166,103.207,102.272,101.362,100.474,99.6083,98.7636,97.9391,97.134,96.3477,95.5793,94.8282,94.0939,93.3756,92.6728,91.985,91.3116,90.6522,90.0063,89.3734,88.7531,88.145,87.5487,86.9638,86.39,85.8269,85.2742,84.7316,84.1988,83.6755,83.1614,82.6563,82.1599,81.6719,81.1922,80.7204,80.2565,79.8001,79.351,78.9092,78.4743,78.0463,77.6248,77.2098,76.8012,76.3987,76.0021,75.6114,75.2265,74.8471,74.4731,74.1044,73.741,73.3825,73.0291,72.6805,72.3366,71.9974,71.6626,71.3323,71.0064,70.6846,70.367,70.0535,69.7439,69.4383,69.1364,68.8383,68.5439,68.253,67.9656,67.6817,67.4012,67.1239,66.85,66.5792,66.3115,66.0469,65.7853,65.5266,65.2708,65.0179,64.7678,64.5204,64.2757,64.0337,63.7942,63.5573,63.3229,63.091,62.8615,62.6343,62.4096,62.1871,61.9668,61.7488,61.533,61.3193,61.1077,60.8983,60.6908,60.4854,60.282,60.0805,59.8809,59.6832,59.4874,59.2934,59.1013,58.9109,58.7222,58.5353,58.35,58.1665,57.9845,57.8043,57.6256,57.4484,57.2729,57.0988,56.9263,56.7553,56.5857,56.4176,56.2509,56.0856,55.9216,55.7591,55.5979,55.438,55.2795,55.1222,54.9662,54.8115,54.658,54.5058,54.3547,54.2049,54.0562,53.9087,53.7623,53.6171,53.473,53.33,53.1881,53.0472,52.9075,52.7688,52.6311,52.4944,52.3588,52.2242,52.0905,51.9578,51.8261,51.6954,51.5656,};
float disCheck2[201]={1091.99,998.112,917.855,848.535,788.121,735.048,688.093,646.285,608.848,575.149,544.673,516.992,491.749,468.646,447.431,427.888,409.833,393.107,377.574,363.115,349.624,337.012,325.198,314.111,303.687,293.872,284.614,275.87,267.599,259.764,252.335,245.28,238.573,232.19,226.108,220.308,214.771,209.481,204.42,199.577,194.936,190.487,186.217,182.118,178.178,174.389,170.743,167.233,163.851,160.59,157.444,154.408,151.476,148.643,145.904,143.255,140.691,138.209,135.805,133.476,131.217,129.027,126.901,124.838,122.835,120.889,118.998,117.159,115.372,113.632,111.94,110.293,108.688,107.126,105.603,104.119,102.673,101.262,99.8857,98.5428,97.2322,95.9527,94.7033,93.483,92.2907,91.1256,89.9867,88.8733,87.7845,86.7195,85.6777,84.6582,83.6604,82.6836,81.7272,80.7905,79.8731,78.9743,78.0936,77.2304,76.3843,75.5548,74.7414,73.9437,73.1612,72.3935,71.6403,70.901,70.1755,69.4632,68.7638,68.0771,67.4026,66.7401,66.0892,65.4498,64.8214,64.2038,63.5967,62.9999,62.4132,61.8362,61.2688,60.7107,60.1617,59.6216,59.0902,58.5673,58.0527,57.5462,57.0476,56.5567,56.0734,55.5975,55.1289,54.6673,54.2127,53.7648,53.3236,52.8889,52.4605,52.0383,51.6222,51.2121,50.8079,50.4093,50.0164,49.6289,49.2469,48.8701,48.4985,48.1319,47.7703,47.4136,47.0617,46.7145,46.3718,46.0337,45.7,45.3706,45.0455,44.7245,44.4077,44.0949,43.786,43.4811,43.1799,42.8825,42.5888,42.2986,42.0121,41.729,41.4493,41.173,40.9,40.6302,40.3637,40.1002,39.8399,39.5826,39.3283,39.0769,38.8284,38.5827,38.3399,38.0997,37.8623,37.6276,37.3954,37.1658,36.9388,36.7142,36.4921,36.2724,36.0551,35.8401,35.6274,35.417,35.2088,35.0028,34.799,};
float disCheck0[201]={145.481,142.234,139.129,136.157,133.309,130.578,127.956,125.438,123.017,120.688,118.446,116.285,114.202,112.192,110.252,108.377,106.566,104.814,103.118,101.477,99.8873,98.3466,96.8527,95.4035,93.9971,92.6316,91.3052,90.0163,88.7633,87.5447,86.3591,85.2053,84.0819,82.9877,81.9217,80.8827,79.8698,78.8819,77.9182,76.9778,76.0598,75.1635,74.288,73.4327,72.597,71.78,70.9812,70.2,69.4359,68.6882,67.9564,67.2401,66.5387,65.8519,65.179,64.5198,63.8738,63.2406,62.6199,62.0112,61.4142,60.8287,60.2542,59.6905,59.1372,58.5941,58.0609,57.5373,57.023,56.5179,56.0217,55.5341,55.0549,54.5839,54.121,53.6658,53.2182,52.778,52.3451,51.9192,51.5002,51.0878,50.6821,50.2827,49.8896,49.5026,49.1216,48.7464,48.3768,48.0129,47.6543,47.3011,46.9531,46.6102,46.2723,45.9392,45.6109,45.2872,44.9681,44.6535,44.3433,44.0373,43.7356,43.4379,43.1443,42.8546,42.5688,42.2868,42.0085,41.7338,41.4627,41.1951,40.9309,40.6701,40.4126,40.1584,39.9073,39.6594,39.4145,39.1726,38.9337,38.6977,38.4645,38.2341,38.0065,37.7815,37.5593,37.3396,37.1224,36.9078,36.6957,36.4859,36.2786,36.0736,35.8709,35.6705,35.4723,35.2763,35.0824,34.8907,34.7011,34.5135,34.3279,34.1443,33.9627,33.783,33.6052,33.4292,33.2551,33.0828,32.9122,32.7435,32.5764,32.411,32.2473,32.0853,31.9249,31.766,31.6088,31.4531,31.2989,31.1462,30.995,30.8453,30.697,30.5502,30.4047,30.2606,30.1179,29.9765,29.8364,29.6977,29.5602,29.424,29.289,29.1553,29.0228,28.8915,28.7614,28.6324,28.5046,28.3779,28.2524,28.1279,28.0046,27.8823,27.7611,27.641,27.5218,27.4037,27.2867,27.1706,27.0555,26.9413,26.8281,26.7159,26.6046,26.4943,26.3848,26.2762,26.1686};
float disCheck1[201]={616.073,591.789,568.968,547.492,527.257,508.165,490.132,473.079,456.935,441.635,427.121,413.338,400.237,387.774,375.907,364.597,353.81,343.513,333.676,324.273,315.277,306.665,298.415,290.506,282.92,275.639,268.647,261.927,255.467,249.251,243.269,237.509,231.958,226.608,221.447,216.468,211.662,207.02,202.536,198.201,194.009,189.955,186.031,182.232,178.553,174.989,171.535,168.186,164.938,161.787,158.729,155.76,152.877,150.077,147.356,144.711,142.139,139.639,137.206,134.839,132.535,130.292,128.108,125.98,123.907,121.887,119.918,117.998,116.126,114.299,112.517,110.778,109.081,107.424,105.806,104.226,102.682,101.174,99.7,98.2593,96.8508,95.4737,94.1269,92.8095,91.5207,90.2597,89.0255,87.8176,86.635,85.4771,84.3433,83.2327,82.1448,81.079,80.0346,79.011,78.0078,77.0243,76.0601,75.1146,74.1873,73.2777,72.3855,71.5101,70.6512,69.8082,68.9809,68.1688,67.3715,66.5886,65.8199,65.065,64.3235,63.5951,62.8795,62.1764,61.4856,60.8066,60.1393,59.4833,58.8385,58.2045,57.5811,56.9681,56.3652,55.7722,55.1889,54.6151,54.0505,53.495,52.9484,52.4104,51.881,51.3598,50.8468,50.3417,49.8445,49.3548,48.8726,48.3978,47.9301,47.4694,47.0157,46.5686,46.1282,45.6942,45.2666,44.8452,44.4298,44.0205,43.617,43.2193,42.8272,42.4406,42.0595,41.6837,41.3131,40.9476,40.5871,40.2316,39.8809,39.535,39.1937,38.857,38.5248,38.1971,37.8736,37.5544,37.2394,36.9285,36.6217,36.3188,36.0198,35.7246,35.4332,35.1455,34.8614,34.5809,34.3039,34.0303,33.7601,33.4933,33.2298,32.9694,32.7123,32.4583,32.2073,31.9594,31.7144,31.4723,31.2331,30.9968,30.7632,30.5324,30.3042,30.0787,29.8559,29.6355,29.4178,29.2025,28.9896};

/***可选择的左右红外拟合曲线***/
/*
float disCheck3[201]={271.919,258.529,246.637,235.993,226.401,217.706,209.78,202.522,195.845,189.68,183.966,178.654,173.7,169.067,164.724,160.643,156.799,153.171,149.741,146.492,143.409,140.48,137.692,135.034,132.499,130.075,127.757,125.537,123.409,121.366,119.403,117.516,115.7,113.951,112.264,110.637,109.067,107.549,106.082,104.662,103.288,101.957,100.667,99.4157,98.2018,97.0232,95.8785,94.7661,93.6845,92.6325,91.6088,90.6122,89.6415,88.6957,87.7739,86.875,85.9981,85.1425,84.3073,83.4918,82.6952,81.9168,81.156,80.4121,79.6846,78.973,78.2766,77.5949,76.9275,76.2739,75.6336,75.0063,74.3914,73.7887,73.1977,72.6181,72.0496,71.4917,70.9442,70.4069,69.8793,69.3612,68.8524,68.3525,67.8614,67.3788,66.9044,66.4381,65.9796,65.5288,65.0853,64.6491,64.2199,63.7976,63.382,62.9729,62.5702,62.1736,61.7832,61.3986,61.0198,60.6467,60.279,59.9168,59.5597,59.2078,58.861,58.519,58.1818,57.8493,57.5214,57.1979,56.8789,56.5641,56.2536,55.9472,55.6448,55.3463,55.0517,54.7609,54.4738,54.1903,53.9104,53.634,53.361,53.0914,52.825,52.5619,52.3019,52.0451,51.7913,51.5405,51.2926,51.0476,50.8054,50.566,50.3293,50.0953,49.8639,49.6352,49.4089,49.1851,48.9638,48.7449,48.5284,48.3142,48.1022,47.8926,47.6851,47.4798,47.2766,47.0756,46.8766,46.6796,46.4846,46.2916,46.1006,45.9114,45.7242,45.5387,45.3551,45.1733,44.9933,44.8149,44.6383,44.4634,44.2901,44.1185,43.9485,43.78,43.6131,43.4478,43.284,43.1216,42.9608,42.8014,42.6434,42.4868,42.3317,42.1779,42.0254,41.8743,41.7245,41.576,41.4288,41.2828,41.1381,40.9946,40.8523,40.7112,40.5713,40.4326,40.295,40.1585,40.0232,39.8889,39.7558,39.6237,39.4927,39.3627,39.2338};
float disCheck2[201]={356.871,337.24,319.907,304.479,290.648,278.17,266.848,256.525,247.069,238.371,230.34,222.901,215.987,209.543,203.521,197.88,192.582,187.597,182.897,178.456,174.254,170.27,166.489,162.893,159.469,156.205,153.09,150.112,147.263,144.534,141.918,139.408,136.996,134.677,132.446,130.297,128.227,126.229,124.301,122.439,120.639,118.899,117.214,115.583,114.003,112.471,110.985,109.544,108.144,106.784,105.463,104.178,102.928,101.713,100.529,99.376,98.2529,97.1583,96.091,95.0502,94.0346,93.0434,92.0758,91.1307,90.2075,89.3053,88.4234,87.5611,86.7177,85.8926,85.0851,84.2948,83.521,82.7631,82.0208,81.2935,80.5806,79.8819,79.1968,78.525,77.866,77.2195,76.5851,75.9624,75.3511,74.751,74.1616,73.5827,73.0139,72.4552,71.906,71.3662,70.8356,70.3139,69.8009,69.2963,68.7999,68.3116,67.8311,67.3582,66.8928,66.4346,65.9836,65.5394,65.102,64.6712,64.2468,63.8287,63.4168,63.0109,62.6108,62.2165,61.8278,61.4446,61.0667,60.6941,60.3267,59.9642,59.6067,59.2541,58.9061,58.5628,58.2239,57.8895,57.5595,57.2337,56.9121,56.5946,56.281,55.9715,55.6657,55.3638,55.0655,54.7709,54.4798,54.1923,53.9082,53.6274,53.35,53.0758,52.8048,52.5369,52.2721,52.0103,51.7515,51.4957,51.2426,50.9924,50.745,50.5002,50.2582,50.0188,49.7819,49.5476,49.3158,49.0865,48.8595,48.635,48.4128,48.1928,47.9752,47.7597,47.5465,47.3354,47.1265,46.9196,46.7148,46.512,46.3112,46.1124,45.9155,45.7205,45.5274,45.3361,45.1467,44.959,44.7731,44.589,44.4065,44.2258,44.0467,43.8693,43.6935,43.5193,43.3467,43.1756,43.0061,42.838,42.6715,42.5064,42.3427,42.1805,42.0197,41.8603,41.7023,41.5456,41.3902,41.2362,41.0835,40.932,40.7818};
*/
 
uint8 GoHead45_fx=0;

 

u8 flag=0;
u8 ir_flag=0;
u8 mm=0;

u8 backflag2=0;
u16 time;

uint8 flag_left                            =0;
uint8 flag_right                           =0; 
extern uint8 zongdianx;
extern uint8 zongdiany;


uint16 after=0;

extern unsigned char actionsequence[100];
extern int action;

extern uint8 STRAIGHT90L;
extern uint8 LV45L;
extern uint8 LT45L;
extern uint8 LV90L;
extern uint8 LV135L;
extern uint8 RV45L;
extern uint8 RT45L;
extern uint8 RV90L;
extern uint8 RV135L;
extern uint8 RT135L;
extern uint8 STRAIGHT45L;
extern uint8 L180L;
extern uint8 R180L;
extern uint8 RT90L;
extern uint8 LT90L;
extern uint8 LT135L;


 
extern float speed_r[278];
extern float speed_l[278];
extern float gyro_r[250];
extern float gyro_l[250];
extern float disCheck3[201];
extern float disCheck2[201];
extern float disCheck0[201];
extern float disCheck1[201];

/*************************************************************/
//此区域参数需修改，根据实际测试数据填入！！！
 
uint16 GusDistance_L_Near=666;       //左斜近距
uint16 GusDistance_R_Near=1635;       //右斜近距

uint16 GusDistance_L_Mid=433;         //左斜中距
uint16 GusDistance_R_Mid=850;        //右斜中距

uint16 GusDistance_L_Far=200;         //左斜远距
uint16 GusDistance_R_Far=1085;         //右斜远距


uint16 GusDistance_FL_Near =910;     //左前近距
uint16 GusDistance_FR_Near =570;     //右前近距

uint16 GusDistance_FL_Far =355;      //左前远距
uint16 GusDistance_FR_Far =245;       //右前远距

float left_distance =20;
float right_distance =80;

float left_yuzhi =100;
float right_yuzhi =100;
 
static int16 angle_temp;   //转弯角度校正基准
uint8 after_turn=0;



//调参

uint16 DisFrX = 500;
uint16 DisFlX = 500;
uint16 DisRX = 200;
uint16 DisLX = 200;

/*******************************************************************************/


float ADCmv(int AD)
{
  float mvdianya;
  mvdianya=AD*3300/4096;
}

/*脉冲测试函数*/
void testEncoder(void)
{
    __GmLeft.uiPulseCtr =0;   
        __GmRight.uiPulseCtr =0;

 __GucMouseState   = __GOAHEAD;
    __GiMaxSpeed      =   SEARCHSPEED;       
    __GmRight.uiPulse =   10 * ONEBLOCK;
    __GmLeft.uiPulse  =   10 * ONEBLOCK;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
     GuiSpeedCtr=__SPEEDUP;
     
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   
 mouseStop();
    while(1);
}
/*********************************************************************************************************
** Function name:       __delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __delay (uint32  uiD)
{
    for (; uiD; uiD--);
}




void Delay_us(u32 Nus)   
{    
	 SysTick->LOAD=Nus*9;          		//ê±???ó??         
	 SysTick->CTRL|=0x01;             //?aê?μ1êy       
	 while(!(SysTick->CTRL&(1<<16))); //μè′yê±??μ?′?    
	 SysTick->CTRL=0X00000000;        //1?±???êy?÷   
	 SysTick->VAL=0X00000000;         //??????êy?÷        
}
/*********************************************************************************************************
** Function name:       PIDInit
** Descriptions:        PID初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void PIDInit(void) 
{  
    __GmLPID.usEncoder_new = 32768;
    //__GmLPID.usEncoder_last = 65535;
    __GmLPID.usFeedBack = 0 ;  //速度反馈值
    __GmLPID.sFeedBack = 0 ;
    
    __GmRPID.usEncoder_new = 32768;
    //__GmRPID.usEncoder_last = 65535;
    __GmRPID.usFeedBack = 0 ;  //速度反馈值
    __GmRPID.sFeedBack = 0 ;
    
    __GmSPID.sRef = 0 ;        //速度设定值 
    __GmSPID.sFeedBack = 0 ;        
    __GmSPID.sPreError = 0 ;   //前一次，速度误差,,vi_Ref - vi_FeedBack 
    __GmSPID.sPreDerror = 0 ;   //前一次，速度误差之差，d_error-PreDerror; 
        
    __GmSPID.fKp = __KP; 
    __GmSPID.fKi = __KI;
    __GmSPID.fKd = __KD; 
       
    __GmSPID.iPreU = 0 ;      //电机控制输出值 
    
    __GmWPID.sRef = 0 ;        //速度设定值 
    __GmWPID.sFeedBack = 0 ;       
    __GmWPID.sPreError = 0 ;   //前一次，速度误差,,vi_Ref - vi_FeedBack 
    __GmWPID.sPreDerror = 0 ;   //前一次，速度误差之差，d_error-PreDerror; 
    
    __GmWPID.fKp = __KP;  //30
    __GmWPID.fKi = __KI;  //0.1,0.01
    __GmWPID.fKd = __KD; 
       
    __GmWPID.iPreU = 0 ;      //电机控制输出值 
    
}

/*********************************************************************************************************
** Function name:       __SPIDContr
** Descriptions:        直线PID控制
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __SPIDContr(void) 
{ 
    float  error,d_error,dd_error;
    static uint8   K_I=1;
    error = __GmSPID.sRef - __GmSPID.sFeedBack; // 偏差计算
    d_error = error - __GmSPID.sPreError; 
    dd_error = d_error - __GmSPID.sPreDerror;
    if(error> Deadband)
      error -= Deadband;
    else if(error < -Deadband)
      error += Deadband;
    else
      error = 0;
    if((error > error_IMAX)||(error < -error_IMAX))
      K_I=0;
    else
      K_I=1;
    
    __GmSPID.sPreError = error; //存储当前偏差 
    __GmSPID.sPreDerror = d_error;
    
    __GmSPID.iPreU += (int16)(  __GmSPID.fKp * d_error + K_I*__GmSPID.fKi * error  + __GmSPID.fKd*dd_error); 
}
/*********************************************************************************************************
** Function name:       __WPIDContr
** Descriptions:        旋转方向PID控制
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __WPIDContr(void) 
{ 
    float  error,d_error,dd_error; 
    static uint8   K_I=1;
    error = __GmWPID.sRef + 0.5*GsTpusle_T -  __GmWPID.sFeedBack; // 偏差计算
    d_error = error - __GmWPID.sPreError; 
    dd_error = d_error - __GmWPID.sPreDerror;
    if(error> Deadband)
      error -= Deadband;
    else if(error < -Deadband)
      error += Deadband;
    else
      error = 0;
    if((error > error_IMAX)||(error < -error_IMAX))
      K_I=0;
    else
      K_I=1;
    
    __GmWPID.sPreError = error; //存储当前偏差 
    __GmWPID.sPreDerror = d_error;
    __GmWPID.iPreU += (int16)(  __GmWPID.fKp * d_error + K_I*__GmWPID.fKi * error  + __GmWPID.fKd*dd_error);
        
}

/*********************************************************************************************************
** Function name:      __PIDContr
** Descriptions:        PID控制，通过脉冲数控制电机
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __PIDContr(void)
{
    __SPIDContr();
    __WPIDContr();
    __GmLeft.sSpeed = __GmSPID.iPreU - __GmWPID.iPreU ;
    if(__GmLeft.sSpeed>=0){
     __GmLeft.cDir=__MOTORGOAHEAD; 
    if( __GmLeft.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmLeft.sSpeed = U_MIN;
    }
    else{
      __GmLeft.cDir=__MOTORGOBACK;
      __GmLeft.sSpeed *=-1; 
    if( __GmLeft.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmLeft.sSpeed = U_MIN;
    }
      
    __GmRight.sSpeed = __GmSPID.iPreU + __GmWPID.iPreU ;
    if(__GmRight.sSpeed>=0){
     __GmRight.cDir=__MOTORGOAHEAD; 
    if( __GmRight.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmRight.sSpeed = U_MIN;
    }
    else{
      __GmRight.cDir=__MOTORGOBACK;
      __GmRight.sSpeed *=-1; 
    if( __GmRight.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmRight.sSpeed = U_MIN;
    }
    __rightMotorContr();
    __leftMotorContr();
    
}
/*********************************************************************************************************
** Function name:       __Encoder
** Descriptions:        采集编码器输出的脉冲
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/



float _jueduizhi(float x)
{
 if(x<0)x=-x;
 return x;
}

uint8 start_firstge=1;
uint8 start_A=0,start_B,start_C,start_D=0;
uint8 quzabo=0;

uint8 start_speedr=0;
uint8 start_speedl=0;
uint8 start_LT45V=0;
uint8 start_RT45V=0;
uint8 start_LT135L=0;
uint8 start_RT135L=0;
uint8 start_LT90V=0;
uint8 start_RT90V=0;
uint8 start_LT180=0;
uint8 start_RT180=0;




float zhixianjuli=0;
float bianmaqijiaodu=0;
uint8 zanshi=0;
static uint8 speedcun[256];
static u16 codernumber=0;
void __Encoder(void)
{   
  uint8 cun_flag=0;
  
    static u16 Dir_L;
    static u16 Dir_R;
	
   
    __GmLPID.usEncoder_new = TIM_GetCounter(TIM2);
    __GmRPID.usEncoder_new = TIM_GetCounter(TIM3);
	
    Dir_R=TIM3->CR1;
    Dir_R=(Dir_R&0x0010)>>4;	
	
    Dir_L=TIM2->CR1;
    Dir_L=(Dir_L&0x0010)>>4;	
		
	if(Dir_L==1)//向下计数  向后退
	{
             __GmLPID.usFeedBack = 32768 - __GmLPID.usEncoder_new;
            // if(__GmLPID.usFeedBack>300)
            //   __GmLPID.usFeedBack=76;
             //__GmLPID.usEncoder_last = __GmLPID.usEncoder_new;
             TIM_SetCounter(TIM2, 32768);
             __GmLeft.uiPulseCtr += __GmLPID.usFeedBack;	
	     __GmLeft.cRealDir = __MOTORGOBACK;
             __GmLPID.sFeedBack= -1*__GmLPID.usFeedBack;		
	}
	else
	{   
            __GmLPID.usFeedBack = __GmLPID.usEncoder_new - 32768;
            TIM_SetCounter(TIM2, 32768);
           // if(__GmLPID.usFeedBack>300)
            //   __GmLPID.usFeedBack=76;
            codernumber+=__GmLPID.usFeedBack;
            __GmLeft.uiPulseCtr += __GmLPID.usFeedBack;
	    __GmLeft.cRealDir = __MOTORGOAHEAD;
            __GmLPID.sFeedBack= __GmLPID.usFeedBack; 
	}
	
	if(Dir_R==1)//向下计数  前进
	{
            __GmRPID.usFeedBack = 32768 - __GmRPID.usEncoder_new;
           // if(__GmRPID.usFeedBack>300)
           //   __GmRPID.usFeedBack=76;
            TIM_SetCounter(TIM3, 32768);
            __GmRight.uiPulseCtr += __GmRPID.usFeedBack;
	    __GmRight.cRealDir = __MOTORGOAHEAD;
	    __GmRPID.sFeedBack = __GmRPID.usFeedBack;
        }  
	else
	{ 
            __GmRPID.usFeedBack = __GmRPID.usEncoder_new - 32768;
            TIM_SetCounter(TIM3, 32768);
           // if(__GmRPID.usFeedBack>300)
           //   __GmRPID.usFeedBack=76;
            __GmRight.uiPulseCtr += __GmRPID.usFeedBack;   
            __GmRight.cRealDir = __MOTORGOBACK;
            __GmRPID.sFeedBack = -1*__GmRPID.usFeedBack;  
           
	}
	__GmSPID.sFeedBack = (__GmRPID.sFeedBack + __GmLPID.sFeedBack)/2 ;
        __GmWPID.sFeedBack = (__GmRPID.sFeedBack - __GmLPID.sFeedBack)/2 ;	
    /*    if(quzabo == 0){__GmRPID.sFeedBack = __GmLPID.sFeedBack = 0;}
        //__GmSPID.sFeedBack = (__GmRPID.sFeedBack + __GmLPID.sFeedBack)/2 ;
        //__GmWPID.sFeedBack = (__GmRPID.sFeedBack - __GmLPID.sFeedBack)/2 ;    //zzzzzz  yuanlai 2
    
        __GmSPID.sFeedBack = (0.5448*__GmRPID.sFeedBack + 0.5448*__GmLPID.sFeedBack)/2 ;//0.52044
       
        if( __GmSPID.sFeedBack>=50)
        {
          __GmWPID.sFeedBack = (0.5448*(__GmRPID.sFeedBack-0.8) - 0.5448*__GmLPID.sFeedBack)/7.03 ; //6.8cm轮子间距
        }
        else
        __GmWPID.sFeedBack = (0.5448*(__GmRPID.sFeedBack) - 0.5448*__GmLPID.sFeedBack)/7.03 ; //6.8cm轮子间距
     
        zhixianjuli = zhixianjuli +  __GmSPID.sFeedBack*0.001;
       bianmaqijiaodu = bianmaqijiaodu + __GmWPID.sFeedBack*0.001;
      
       
       
       
        if(start_firstge == 1)
        {
         if(zhixianjuli>=13.2){start_firstge = 0;zhixianjuli = 0;}
        }
        
        
        if(start_firstge == 0)
        {
          
         
           if((zhixianjuli<=9)&&(zhixianjuli>0)){start_A=1;}else {start_A=0;}
           if(zhixianjuli>9){start_B=1;}else {start_B=0;}
           if(zhixianjuli>=18){zhixianjuli = zhixianjuli - 18;}
          
        }
       
      */
        
        
       
        
}




void __irSendFreq (int8  __cNumber)
{
    switch (__cNumber) 
    {
      case 1: /*前方红外*/                                                            
          GPIO_SetBits(GPIOA,GPIO_Pin_5);  //前方左 开
           break;
  
      case 2:  /*左右红外*/                                                         
          GPIO_SetBits(GPIOC,GPIO_Pin_13); //前方右 开
          break;  
      case 3:                                                         
          GPIO_SetBits(GPIOA,GPIO_Pin_3); //左 开
          break; 
      case 4:                                                         
          GPIO_SetBits(GPIOC,GPIO_Pin_2); //右 开
          break;           
      default:
          break;
    }
}

void sensorDebug (void)
{
    zlg7289Download(2, 0, 0, __GucDistance[__LEFT  ]);//传感器左
    zlg7289Download(2, 1, 0, __GucDistance[__FRONTL]);//传感器前左
    zlg7289Download(2, 2, 0, __GucDistance[__FRONTR]);//传感器前右    
    zlg7289Download(2, 3, 0, __GucDistance[__RIGHT ]);//传感器右
}

/*********************************************************************************************************
** Function name:       __rightMotorContr
** Descriptions:        右直流电机驱动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __rightMotorContr(void)
{
    switch (__GmRight.cDir) 
    {
    case __MOTORGOAHEAD:                                                
      TIM_SetCompare1(TIM1,__GmRight.sSpeed);
      TIM_SetCompare2(TIM1,0);
        break;

    case __MOTORGOBACK:                                                 
      TIM_SetCompare1(TIM1,0);
      TIM_SetCompare2(TIM1,__GmRight.sSpeed);
        break;
		
    case __MOTORGOSTOP:                                                   
      TIM_SetCompare1(TIM1,0);		
      TIM_SetCompare2(TIM1,0);
        break;

    default:
        break;
    }
}

/*********************************************************************************************************
** Function name:       __leftMotorContr
** Descriptions:        左直流电机驱动
** input parameters:    __GmLeft.cDir :电机运行方向
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __leftMotorContr(void)
{
    switch (__GmLeft.cDir) 
    {
    case __MOTORGOAHEAD:                                            
      TIM_SetCompare4(TIM1,0);                                          
      TIM_SetCompare3(TIM1,__GmLeft.sSpeed);
        break;

    case __MOTORGOBACK:                                                 
      TIM_SetCompare4(TIM1,__GmLeft.sSpeed);                                          
      TIM_SetCompare3(TIM1,0);
        break;
		
    case __MOTORGOSTOP:                                                 
      TIM_SetCompare3(TIM1,0);                                          
      TIM_SetCompare4(TIM1,0);
        break;
		
    default:
        break;
    }
}

/*********************************************************************************************************
** Function name:       __SpeedUp
** Descriptions:        电脑鼠加速程序
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __SpeedUp (void)
{
    uint16 Speed;
    Speed=__GmSPID.sFeedBack;
    if(__GiMaxSpeed>__VmaxSpeed)
    {
       __GiMaxSpeed=__VmaxSpeed;
    }
    if(__GmSPID.sRef<__GiMaxSpeed){
      if(Speed >=__GmSPID.sRef)
      {
        __GmSPID.sRef=__GmSPID.sRef+10;
      }
    }   
}
/*********************************************************************************************************
** Function name:       __SpeedDown
** Descriptions:        电脑鼠减速程序
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __SpeedDown (void)
{
    uint16 Speed;
    Speed=__GmSPID.sFeedBack;
    if(__GmSPID.sRef>=MINSPEED)
    {
      if(Speed <=__GmSPID.sRef+3)
      {
       __GmSPID.sRef=__GmSPID.sRef-2;
      }
    }
}


/*********************************************************************************************************
** Function name:       startCheck
** Descriptions:        读取按键
** input parameters:    无
** output parameters:   无
** Returned value:      true:  按键已按下
**                      false: 按键未按下
*********************************************************************************************************/
uint8 startCheck (void)
{
  if (GPIO_ReadInputDataBit(GPIOC, __START) == 0) {
        __delay(50);
        while(GPIO_ReadInputDataBit(GPIOC, __START) == 0);
        return(true);
    }else {
        return(false);
    }
}




void TIM2_IROHandler(void)
{
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
}


void TIM3_IRQHandler(void)
{
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
}

u32 GyroscopeDebug[8] = {0};
/*
SourceDate : 采样值存储地址
NewDate : 新采样值
Average ：返回值
*/
void GyroscopeFilter(u32 NewDate,u32 *SourceDate,u32 *Average)
{
   
  unsigned char i = 0;
  u32 NumMax,NumMin = 0;
  unsigned char MaxCount,MinCount = 0;
  u32 Sum = 0;
  for(i = 0;i < 7;i++)
   {
     *(SourceDate + i) = *(SourceDate + i + 1);   
   }
   *(SourceDate + 7) = NewDate;
  NumMax = *SourceDate;
  NumMin = *SourceDate;
  MaxCount = 0;
  MinCount = 0;
  for(i = 1;i < 8;i++)
  {
    if(*(SourceDate + i) > NumMax)        //如果大于当前最大值 
    {
      NumMax = *(SourceDate + i);
      MaxCount = i;
    }
    else if(*(SourceDate + i) < NumMin )
    {
      NumMin = *(SourceDate + i);
      MinCount = i;
    }
    else
    {}
  }
  if(NumMax == NumMin)
  {
    *Average = NumMax;
  }
  else 
  {
    Sum = 0;
    for(i = 0;i < 8;i++)
    {
      if((i != MaxCount)&&(i != MinCount))
      {
        Sum += *(SourceDate + i);
      }
    }
    *Average = Sum/6;
  }
  
}
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
float ADCMVdianyar=0;
float ADCMVdianya=0;
float  ADCMVdianyafl=0;
float  ADCMVdianyafr=0;
static float leftdis=0;
static float rightdis=0;
static float fldis=0;
static float frdis=0;

void TIM6_IRQHandler(void)
{
  
     //调参
    int bijiaozhi1=200;//右方红外的
    int quzhenzhi1=15;
    int bijiaozhi2=700;//左前红外的
    int quzhenzhi2=16;
    int bijiaozhi3=700;//右前红外的
    int quzhenzhi3=16;
    int bijiaozhi4=200;//左方红外的
    int quzhenzhi4=15;
 
    static uint8 ucState = 0;
    static uint16 integer=0;      //红外距离表整数部分
    static float decimal=0;      //红外距离表小数部分
    static uint16 integerr=0;
    static float decimalr=0; 
    
    static uint16 integerfl=0;
    static float decimalfl=0; 
    static uint16 integerfr=0;
    static float decimalfr=0;

    //unsigned char i = 0;
    TIM_ClearFlag(TIM6,TIM_FLAG_Update);
    GyroscopeFilter(ADC_ConvertedValue[4],Angle_TLY,&Angle_TLY_Average);

     switch(ucState)
    {
        case 0:
            Sunlight[3] = ADC_ConvertedValue[3];
            ucState++;
            __irSendFreq (4);
            break;
            
        case 1:
            ucIRCheck[3] = ADC_ConvertedValue[3]-Sunlight[3];
             if(ucIRCheck[3]>35000)
                ucIRCheck[3] = 0;  
            ADCMVdianya=ADCmv(ucIRCheck[3]);
 
            rightdis= rightd(ADCMVdianya,bijiaozhi1,quzhenzhi1);//
                   
            GPIO_ResetBits(GPIOC,GPIO_Pin_2); 
            if(ucIRCheck[3]>GusDistance_R_Far)
            {
                __GucDistance[__RIGHT]  |= 0x01;
            }         
            else
            {
                __GucDistance[__RIGHT]  &= 0xfe;              
            } 
          
            if(ucIRCheck[3]>GusDistance_R_Mid)
            {
                __GucDistance[__RIGHT]  |= 0x02;
            }
          
            else
            {
                __GucDistance[__RIGHT]  &= 0xfd;
            }  
          
            if(ucIRCheck[3]>GusDistance_R_Near)
            {
                __GucDistance[__RIGHT]  |= 0x04;
            }
          
            else
            {
                __GucDistance[__RIGHT]  &= 0xfb;
            }
            ucState++;
           // __irSendFreq (1);
            break;
            
        case 2:
            Sunlight[0] = ADC_ConvertedValue[0];
            ucState++;
            __irSendFreq (1);
            break;
            
        case 3:
           ucIRCheck[0] = ADC_ConvertedValue[0]-Sunlight[0];
             ADCMVdianyafl=ADCmv(ucIRCheck[0]);
            if(ucIRCheck[0]>30000)
                ucIRCheck[0]=0;    
            
           fldis=flid(ADCMVdianyafl,bijiaozhi2,quzhenzhi2);//线性取值
            
            
            GPIO_ResetBits(GPIOA,GPIO_Pin_5);
          
            if(ucIRCheck[0]>GusDistance_FL_Far)
            {
                __GucDistance[__FRONTL]  |= 0x01;
            }
            else
            {
                __GucDistance[__FRONTL] &= 0xfe;
            }  
            if(ucIRCheck[0]>GusDistance_FL_Near)
            {
                __GucDistance[__FRONTL]  |= 0x02; 
            } 
            else
            {
                __GucDistance[__FRONTL]  &= 0xfd;
            }
            //__irSendFreq (2);
            ucState++;
            break;
            
        case 4:
            Sunlight[1] = ADC_ConvertedValue[1];
            ucState++;
            __irSendFreq (2);
            break;
            
        case 5:
            ucIRCheck[1] = ADC_ConvertedValue[1]-Sunlight[1];
            ADCMVdianyafr=ADCmv(ucIRCheck[1]);
            if(ucIRCheck[1]>30000)
                ucIRCheck[1]=0;   
            
           frdis=frid(ADCMVdianyafr,bijiaozhi3,quzhenzhi3);//
            
            GPIO_ResetBits(GPIOC,GPIO_Pin_13);
            if(ucIRCheck[1]>GusDistance_FR_Far)
            {
                __GucDistance[__FRONTR]  |= 0x01;
            }
            else
            {
                __GucDistance[__FRONTR] &= 0xfe;
            } 
          
            if(ucIRCheck[1]>GusDistance_FR_Near)
            {
                __GucDistance[__FRONTR]  |= 0x02;
            }
            else
            {
                __GucDistance[__FRONTR]  &= 0xfd;
            }          
            if((ucIRCheck[0]>GusDistance_FL_Far)&&(ucIRCheck[1]<GusDistance_FR_Far))
            {
                GucGoHead =1;
            }
            else if((ucIRCheck[0]<GusDistance_FL_Far)&&(ucIRCheck[1]>GusDistance_FR_Far))
            {
                GucGoHead1 =1;
            }
            else if((ucIRCheck[0]>GusDistance_FL_Far)&&(ucIRCheck[1]>GusDistance_FR_Far))
            {
                GucGoHead2 =1;
            }
            else
            {
                GucGoHead =0;
                GucGoHead1 =0;
                GucGoHead2 =0;
            }
            ucState++;
            //__irSendFreq (3);
            break;
            
        case 6:
            Sunlight[2] = ADC_ConvertedValue[2];
            ucState++;
            __irSendFreq (3);
            break;
            
        case 7:
            ucIRCheck[2] = ADC_ConvertedValue[2]-Sunlight[2];
             if(ucIRCheck[2]>30000)
                ucIRCheck[2]=0;  
            ADCMVdianyar=ADCmv(ucIRCheck[2]);
            
           leftdis=leftd(ADCMVdianyar,bijiaozhi4,quzhenzhi4); //线性取值
           
            GPIO_ResetBits(GPIOA,GPIO_Pin_3);
            if(ucIRCheck[2]>GusDistance_L_Far)
            {
                __GucDistance[__LEFT]   |= 0x01;
            }
            else
            {
                __GucDistance[__LEFT]   &= 0xfe;
            }
          
            if(ucIRCheck[2]>GusDistance_L_Mid)
            {
                __GucDistance[__LEFT]   |= 0x02;
            }
            else
            {
                __GucDistance[__LEFT]   &= 0xfd;
            }
          
            if(ucIRCheck[2]>GusDistance_L_Near)
            {
                __GucDistance[__LEFT]   |= 0x04;
            }
            else
            {
                __GucDistance[__LEFT]   &= 0xfb;
            }
            if((__GucDistance[__FRONTR] & 0x02)&&(__GucDistance[__FRONTL] & 0x02))
            {
              GucFrontJinju = 1;
            }
            else 
            {
              GucFrontJinju = 0;
            }
            
            ucState++;
            //__irSendFreq (4);
            //GyroscopeFilter(Angle_TLY,0);
            
            
            
            break;
            
        default:
            break;
    }
    
    ucState = ucState % 8;
}



/*********************************************************************************************************
** Function name:       SysTick_Handler
** Descriptions:        定时中断扫描。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
uint8 end_speedr=0;
uint8 end_speedl=0;
uint8 end_RT45V=0;
uint8 end_LT45V=0;
uint8 end_RT135L=0;
uint8 end_LT135L=0;
uint8 end_LT90V=0;
uint8 end_RT90V=0;
uint8 end_LT180=0;
uint8 end_RT180=0;

uint16 start_flag=0;
uint16 start_flag1=0;
uint16 start_RT45Vflag=0;
uint16 start_LT45Vflag=0;
uint16 start_RT135Lflag=0;
uint16 start_LT135Lflag=0;
uint16 start_LT90Vflag=0;
uint16 start_RT90Vflag=0;
uint16 start_LT180flag=0;
uint16 start_RT180flag=0;

uint8 zuihouyige=0;

uint16 lingshiL,lingshiR,lingshizuixiaoL,lingshizuixiaoR=0;
uint8 lingshichengliL,lingshichengliR=0;

extern uint8 zijiaozheng0;


static float zijiaozheng=0;
uint16 zijiaozheng_time=0;
static float sum_zijiaozheng=0;
uint8 zijiaozheng_flag=1;

uint8 Start_goback=0;//返回点编码器清零标志

static u16 qidianclock=0;
static uint8 use_gyro=1;
extern uint8 GYROInit;
static uint8 GYROSTAT=1;
static float GYROTim=0;
static float GYRONum=0;
static float GYROZero=0;

void SysTick_Handler(void)
{  
  
  //调参
  float leftbiao=left_distance;    //标定值
  float rightbiao=right_distance;
  float leftyu=left_yuzhi;
  float rightyu=right_yuzhi;
  
  
  
    static int8 n = 0,m=0 ,k=0,l=0,a=0,b=0,c=0,w=0;
    static u16  t=0, s=0;
    static u32  encoder=0;
    uint16 Sp;
    if(zijiaozheng0==1)
        {
        if(zijiaozheng_flag)
        {  
         
          sum_zijiaozheng += Angle_TLY_Average;
          zijiaozheng_time++;
          if(zijiaozheng_time==3000){zijiaozheng=sum_zijiaozheng/3000;zijiaozheng_flag=0;jiao=0;}
          
        }
        
        }
    __Encoder(); 
    TIM6_IRQHandler();
    Sp=__GmSPID.sFeedBack;
    encoder+=Sp;
    t++;
    s++;
    if(t == 1000)
    {
        t = 0;
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        if(s == 2000)
        {
            s = 0;
            GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        }
    }


        
        switch (__GmRight.cState) {
     case __MOTORSTOP:                                                   /*  停止，同时清零速度和脉冲值  */
              __GmRight.uiPulse    = 0;
              __GmRight.uiPulseCtr = 0;
              __GmLeft.uiPulse    = 0;
              __GmLeft.uiPulseCtr = 0;
              break;
      
          case __WAITONESTEP:                                                 /*  暂停一步                    */
              __GmRight.cState = __MOTORRUN;
              
              GsTpusle_T=dis(leftdis, rightdis, leftbiao, leftyu, rightbiao, rightyu);//
      
              break;
 
          case __MOTORRUN:                                                    /*  电机运行                    */
            if (__GucMouseState == __GOAHEAD)                                 /*  根据传感器状态微调电机位置  */
            {                              
                  //搜索冲刺直线矫正
                  GsTpusle_T=disr(leftdis, rightdis, leftbiao, leftyu, rightbiao, rightyu);//
                  if(GuiSpeedCtr==__SPEEDUP)
                  { 
                    k=(k+1)%5;//20
                    if(k==4)
                    __SpeedUp();
                  }
                  else if(GuiSpeedCtr==__SPEEDDOWN)
                  {
                      k=(k+1)%10;
                      if(k==5)
                      __SpeedDown(); 

                  }
                  else;     
            }
            else if (__GucMouseState == __GOAHEAD_45)  //45度矫正
            {       
                GsTpusle_T= dis45(DisFlX,DisFrX); 
                __PIDContr();

                if(GuiSpeedCtr==__SPEEDUP)
                    { 
                      k=(k+1)%10;//20
                      if(k==9)
                      __SpeedUp();
                    }
                else if(GuiSpeedCtr==__SPEEDDOWN)
                    {
                        k=(k+1)%10;
                        if(k==5)
                        __SpeedDown(); 

                    }
                else;
            }   
            else
            {
                  GsTpusle_T = 0;             
                  voltageDetect();      //陀螺仪            
            }                  
            __PIDContr();     
            break;
           
          case 4:
            GsTpusle_T = 0;
            __PIDContr();
            break;
  
           default:
             break;
    }           
}
/*********************************************************************************************************
** Function name:       mazeSearch
** Descriptions:        前进N格
** input parameters:    iNblock: 前进的格数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseGoahead1 (int8  cNBlock)                       //冲刺看前方红外，无坐标更新,当goto end速度大于20时设定速度为20
{
    int8 cL = 0, cR = 0, cCoor = 1,cB;
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    if(cNBlock==1)
    {
        cL = 1;
        cR = 1;
        if(GucFangXiang == GucDirTemp)
        {
           GuiTpusle_LR = 3200;
           GuiTpusle_S = 0;
        }
        else
        {
           GuiTpusle_LR = 3200;
           GuiTpusle_S = 0;
        }
        __GiMaxSpeed = 100;
    }
    else{
        GuiTpusle_LR = 0;
        GuiTpusle_S = 0;
    }
    GucFangXiang = GucDirTemp;
    if(((GmcMouse.cX==7)&&(GmcMouse.cY==7))|| 
         ((GmcMouse.cX==8)&&(GmcMouse.cY==8))||
         ((GmcMouse.cX==7)&&(GmcMouse.cY==8))||
           ((GmcMouse.cX==8)&&(GmcMouse.cY==7))){
       cL = 0;
       cR = 0; 
       GuiTpusle_LR = 0;
       GuiTpusle_S = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr = 14400;       //1182(34mm)
        __GmRight.uiPulseCtr = 14400;//7200
    }
    
    cB=cNBlock;
    __GucMouseState   = __GOAHEAD;
   // __GiMaxSpeed      =   25;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock * ONEBLOCK ;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock * ONEBLOCK ;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    if(cNBlock > 5)
    {
      __GiMaxSpeed=140;
    }
    else if(cNBlock == 5)
    {
      __GiMaxSpeed=132;
    }
    else if(cNBlock == 4)
    {
      __GiMaxSpeed=120;
    }
    else if(cNBlock == 3)
    {
      __GiMaxSpeed=108;
    }
    else if(cNBlock == 2)
    {
      __GiMaxSpeed=100;
    }
    else
     ;
    while (__GmLeft.cState != __MOTORSTOP) {
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  判断是否走完一格            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
                cNBlock--;
                if(cNBlock==0)
                   goto End;
                if(cNBlock<cB-1)//给回速一个时间
                  GuiSpeedCtr=__SPEEDUP;
            } else {
                cCoor = 1;
            }
        }
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if (cNBlock < 2) {
          if(__GmSPID.sFeedBack>100){
              GuiSpeedCtr= 3;
             __GmSPID.sRef=100;
          }  
          if (cL) 
          {                                                       /*  是否允许检测左边            */
            if (ucIRCheck[2]<GusDistance_L_Far)             /*  左边有支路，跳出程序        */
            {                 
                __GmRight.uiPulse = __GmRight.uiPulseCtr  + 6000- GuiTpusle_LR;    //3094(89mm)
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr   + 6000- GuiTpusle_LR;
                while (ucIRCheck[2]<GusDistance_L_Far) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {                                                    /*  左边有墙时开始允许检测左边  */
                if (ucIRCheck[2]>GusDistance_L_Far) {
                    cL = 1;
                }
            }
         if (cR) 
            {                                                       /*  是否允许检测右边            */
            if (ucIRCheck[3]<GusDistance_R_Far)               /*  右边有支路，跳出程序        */
            {                
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 6000- GuiTpusle_LR;
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr + 6000- GuiTpusle_LR;
                while (ucIRCheck[3]<GusDistance_R_Far) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {
                if (ucIRCheck[3]>GusDistance_R_Far) {                   /*  右边有墙时开始允许检测右边  */
                    cR = 1;
                }
            }
        }
   }
    /*
     *  设定运行任务，让电脑鼠走到支路的中心位置
     */
End:     ;//GuiSpeedCtr= 3;
         //__GmSPID.sRef=18;
}


static uint8 Continuous=0;
uint8 final;
void mouseGoahead(int8  cNBlock)                                
{
    int8 cL = 0, cR = 0, cCoor = 1,cB;
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState)                   
    {                  
        cCoor = 0;
    }
    if(cNBlock==1)
    {
        cL = 1;
        cR = 1;
        if(GucFangXiang == GucDirTemp)      
        {
           GuiTpusle_LR =15000;
           GuiTpusle_S  = 0;               
        }
        else
        {
           GuiTpusle_LR = 10000;
           GuiTpusle_S  = 600;
        }
        __GiMaxSpeed = 100;
    }
    else
    {
        GuiTpusle_LR = 0;
        GuiTpusle_S  = 0;
        __GiMaxSpeed = 320;
    }
    GucFangXiang = GucDirTemp;
    if(((GmcMouse.cX==7)&&(GmcMouse.cY==7))|| 
       ((GmcMouse.cX==8)&&(GmcMouse.cY==8))||
       ((GmcMouse.cX==7)&&(GmcMouse.cY==8))||
       ((GmcMouse.cX==8)&&(GmcMouse.cY==7)))
    {
        cL = 0;
        cR = 0;
        GuiTpusle_LR = 0;
        GuiTpusle_S  = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr = 30000;      
        __GmRight.uiPulseCtr= 30000;

    }
    
    cB=cNBlock;
    __GucMouseState   = __GOAHEAD;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock * ONEBLOCK ;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock * ONEBLOCK ;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    if(cNBlock > 5)
    {
        __GiMaxSpeed=130;
    }
    else if(cNBlock == 5)
    {
        __GiMaxSpeed=130;
    }
    else if(cNBlock == 4)
    {
        __GiMaxSpeed=130;
    }
    else if(cNBlock == 3)
    {
        __GiMaxSpeed=130;
    }
    else if(cNBlock == 2)
    {
        __GiMaxSpeed=130;
    }
    else;
       
    while (__GmLeft.cState != __MOTORSTOP)
    {
        if (__GmLeft.uiPulseCtr >= ONEBLOCK)                /*  判断是否走完一格            */
        {                          
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) 
            {
                cNBlock--;
                if(cNBlock==0)
                    goto End;
                if(cNBlock<cB-1)                          //给回速一个时间
                    GuiSpeedCtr=__SPEEDUP;
            } 
            else 
            {
                cCoor = 1;
            }
        }
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK)                /*  判断是否走完一格            */
        {                         
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if(cNBlock < 3)
        {
            if(__GmSPID.sFeedBack>160)
            {              
                 __GmSPID.sRef=160;
              //  GuiSpeedCtr= __SPEEDDOWN;
            }          
        }
        if (cNBlock < 2) 
        {         
            if(__GmSPID.sFeedBack>120)
            {
                GuiSpeedCtr= 3;
                __GmSPID.sRef=2;
            }  
          
            if (cL) 
            {                                                       /*  是否允许检测左边            */
                if (ucIRCheck[2]<GusDistance_L_Far)                 /*  左边有支路，跳出程序        */
                {                 
                    __GmRight.uiPulse = __GmRight.uiPulseCtr  + 27700 - GuiTpusle_LR;    
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr   + 27700 - GuiTpusle_LR;
                    while (ucIRCheck[2]<GusDistance_L_Far) 
                    {                   
                        if ((__GmLeft.uiPulseCtr + 100) > __GmLeft.uiPulse) 
                        {
                            goto End;
                        }
                    }
                }
            } 
            else 
            {                                                    
                if (ucIRCheck[2]>GusDistance_L_Far)                 /*  左边有墙时开始允许检测左边  */
                {
                    cL = 1;
                }
            }
            if (cR) 
            {                                                       /*  是否允许检测右边            */
                if (ucIRCheck[3]<GusDistance_R_Far)                 /*  右边有支路，进入程序        */
                {                
                    __GmRight.uiPulse  = __GmRight.uiPulseCtr + 26500- GuiTpusle_LR;
                    __GmLeft.uiPulse   = __GmLeft.uiPulseCtr  + 26500- GuiTpusle_LR;
                    while (ucIRCheck[3]<GusDistance_R_Far) 
                    {                   
                        if ((__GmLeft.uiPulseCtr + 100) > __GmLeft.uiPulse) 
                        {
                            goto End;
                        }
                    }
                }
            } 
            else 
            {
                if (ucIRCheck[3]>GusDistance_R_Far)                 /*  右边有墙时开始允许检测右边  */
                {                   
                    cR = 1;
                }
            }
        }
   }


End:     ;

}












uint8 C_continuous=0;

static uint8 sssssspp=0;
uint8 qidianwucha=0;

void mouseGoahead_liang (int8  cNBlock)                                    //连续转弯用
{
   
   int8 cL = 0, cR = 0, cCoor = 1,cB;
   __GmLeft.uiPulse =0;          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =0;           
   __GmRight.uiPulseCtr=0;
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    GucFangXiang = GucDirTemp;
    if(((GmcMouse.cX==7)&&(GmcMouse.cY==7))|| 
         ((GmcMouse.cX==8)&&(GmcMouse.cY==8))||
         ((GmcMouse.cX==7)&&(GmcMouse.cY==8))||
           ((GmcMouse.cX==8)&&(GmcMouse.cY==7))){
       cL = 0;
       cR = 0;
       GuiTpusle_LR = 0;
       GuiTpusle_S  = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr = 30000;       //1182(34mm)
        __GmRight.uiPulseCtr = 30000;

    }
    
    cB=cNBlock;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  
   // __GiMaxSpeed      =   25;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock * ONEBLOCK ;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock * ONEBLOCK ;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    
    if(cNBlock > 5)
    {
      __GiMaxSpeed=240;
    }
    else if(cNBlock == 5)
    {
      __GiMaxSpeed=240;
    }
    else if(cNBlock == 4)
    {
      __GiMaxSpeed=240;
    }
    else if(cNBlock == 3)
    {
      __GiMaxSpeed=240;
    }
    else if(cNBlock == 2)
    {
      __GiMaxSpeed=240;
    }
    else  __GiMaxSpeed=180;
    
    
    while (__GmLeft.cState != __MOTORSTOP)
    {
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) 
        {                          /*  判断是否走完一格            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
                cNBlock--;
                if(cNBlock==0)
                   goto End;
             /*   if(cNBlock<cB-1)//给回速一个时间
                  GuiSpeedCtr=__SPEEDUP;*/
            } else {
                cCoor = 1;
            }
        }
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
              if(cNBlock < 3)
        {
            if(__GmSPID.sFeedBack>220)
            {              
                __GiMinSpeed=200;
                GuiSpeedCtr= __SPEEDDOWN;
            }          
        }
        
        
        if (cNBlock < 2)   //2//45度时，最后一格走完这么多脉冲后开始检测柱子
        {
           if(__GmSPID.sFeedBack>200)
            {
               __GmSPID.sRef=200;
                GuiSpeedCtr= __SPEEDDOWN;
            }
          while(1)
          {
           if (cL) 
           {                                                       /*  是否允许检测左边            */
              if  ((__GucDistance[__LEFT]  & 0x01)==0)
              {                 /*  左边有支路，跳出程序        */
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                        /*  左边有墙时开始允许检测左边  */
              if (ucIRCheck[2]>GusDistance_L_Far) 
              {
                  cL = 1; 
              }
           }//20550
          if (cR) 
          {                                                       /*  是否允许检测右边            */
              if ((__GucDistance[__RIGHT]  & 0x01)==0)
              {               /*  右边有支路，跳出程序        */
                goto End;
              }
          }
          else 
          {
              if (ucIRCheck[3]>GusDistance_R_Far)
              {                       
                  cR = 1;
              }
          }
         }
        }

   }
    /*
     *  设定运行任务，让电脑鼠走到支路的中心位置
     */
End:     ;
         //GuiSpeedCtr= 3;
         //__GmSPID.sRef=23;
}

void mouseTurnleft_90(void)
{
  
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                             /*  方向标记                    */
             
   __GmSPID.sRef=100;           ///120
   __GmWPID.sRef=80;//70
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>85200)   
       {
         break;
       } 
      
   }
    
   __GmWPID.sRef=0;
   __GucMouseState   = __GOAHEAD;
 		//mouseStop();
 		//while(1)
			;	 
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =2000;          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =2000;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);	
 
   __GucMouseState   = __TURNLEFT;
 	
   GuiSpeedCtr=__SPEEDUP;
  
   // __GmRight.cState = __MOTORSTOP;       
   // __GmLeft.cState  = __MOTORSTOP; 
      __GmRight.uiPulse    = 0;
              __GmRight.uiPulseCtr = 0;
              __GmLeft.uiPulse    = 0;
              __GmLeft.uiPulseCtr = 0;
             

}

void mouseTurnleft_90_t(void)
{
  int cL=0,cR=0;
  static int i=0;
  i++;
  //  mouseStop();while(1);
   __GmSPID.sRef=200;
   // mouseStop();while(1);
     piancha_l = (rightdis - 55)/10;
   if(rightdis>125){piancha_l = 0;}//左转右边没墙
   if(piancha_l > 2){piancha_l = 2;}
    if(piancha_l < -2){piancha_l = -2;}
   if(usepiancha==0)piancha_l = 0;
   
   __GmLeft.uiPulse =16500-piancha_l*2886.67;    //9000    //14400      
    __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse=16500-piancha_l*2886.67;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   
 
   GW=0;
   time=0;   
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
   
  
   __GmWPID.sRef=120;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>84000) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
  // if(i==4){}
 
  //
   __GmSPID.sRef=200;
   __GmWPID.sRef=0;
   //mouseStop();
  // while(1);
    
     
   
      // mouseStop();while(1);
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
  
     
        __GmLeft.uiPulse =43500 ;    //9000    //14400      
    __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse=43500 ;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
//   mouseStop();while(1);
 
    
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
End:
    {//GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;}

}

void mouseTurnleft_90_v(void)
{     __GmSPID.sRef=200;
   uint8 cL = 0,cR=0;
   static int i=0;
   i++;
   __GmLeft.uiPulse =7400;    //9000 7500         
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =7400;           
   __GmRight.uiPulseCtr=0;

   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
   
 //    mouseStop();while(1);
   __GmWPID.sRef=120;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>84000) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;
   //GuiSpeedCtr=__SPEEDUP;
 
      __GucMouseState = __GOAHEAD_45;
   
   GuiSpeedCtr = __SPEEDUP ;
   __GmLeft.uiPulse = 22000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 22000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    GuiSpeedCtr=__SPEEDUP;
    //__GmRight.cState = __MOTORSTOP;       
    //__GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
   End:{
    GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}
}




void mouseTurnleft_135_t(void)//wan
{
   int8 cR = 0,cL=0;
   static int i=0;
   i++;__GmSPID.sRef=200;
  
     piancha_l = (rightdis - 55)/10;
   if(rightdis>125){piancha_l = 0;}//左转右边没墙
   if(piancha_l > 2){piancha_l = 2;}
    if(piancha_l < -2){piancha_l = -2;}
   if(usepiancha==0)piancha_l = 0;
   
   __GmLeft.uiPulse =20300-piancha_l*2886.67;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =20300-piancha_l*2886.67;           
   __GmRight.uiPulseCtr=0;
   GW=0;
   time=0;   
   
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GucMouseDir     = (GucMouseDir + 5) % 8;                          
   
    __GucMouseState   = __TURNLEFT;
   __GmWPID.sRef=120;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>132000) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
 

   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;
      __GucMouseState = __GOAHEAD_45;
   // mouseStop(); while(1);
      
      
   //__GmSPID.sRef=160;
   //.GuiSpeedCtr = __SPEEDUP ;
   __GmLeft.uiPulse =32000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =32000;           
   __GmRight.uiPulseCtr=0;
   __GmRight.cState  = __MOTORRUN;
   __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);   
   //GuiSpeedCtr=__SPEEDUP;
   // __GmRight.cState = __MOTORSTOP;       
   // __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
    __GmRight.uiPulse =  0;
    __GmLeft.uiPulse = 0;
End:;
}


void mouseTurnleft_135_v(void)
{
   int8 cL = 0, cR = 0;
   __GmLeft.uiPulse =27000;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =27000;           
   __GmRight.uiPulseCtr=0;
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;  
   __GucMouseState   = __TURNRIGHT; 
  // mouseStop();while(1);
   GucMouseDir     = (GucMouseDir + 5) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=140;
   __GmWPID.sRef=100;
   
   //   
     while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>132000) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
   
    __GmLeft.uiPulse =27000;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =27000;           
   __GmRight.uiPulseCtr=0;
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
//   mouseStop();while(1);  
   GoHead45_fx=0;
   GoHead45_flag=0;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=160;   
   __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
 
   
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
   __rightMotorContr();
   __GmLeft.sSpeed = 0;
   __leftMotorContr();
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   End: 
  {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
  }
}

void mouseTurnright_90(void)
{  
   //i++;
//    mouseStop();
//    while(1);
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=100;
   __GmWPID.sRef=-80;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>41200*1.8)   
       {
         break;
       } 
      
   }

   __GmWPID.sRef=0;     
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =14400;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =14400;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  //if(i==6)
  //{
    //mouseStop();
    //while(1);
  //}
  // __GucMouseState   = __TURNRIGHT;
   
//   mouseStop();
//    while(1);  
    
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
  //mouseStop();
    //while(1);
    //if(i==)
    //{
    //  mouseStop();
    // while(1);
    //}
   
}
void mouseTurnright_90_t(void)
{  
  int cR=0,cL=0;
  __GucMouseState   = __GOAHEAD;
  static int ii=0;ii++;
  
      piancha_r = (leftdis - 40)/10;
   if(leftdis > 120){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
   if(usepiancha==0) piancha_r=0;
  
     __GmLeft.uiPulse =20000-piancha_r*2886.67;    //9000    //14400      
    __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse=20000-piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=160;
   __GmWPID.sRef=-100;
  while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>84000) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
//   if(ii==2){}
   
   __GmWPID.sRef=0;     
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   after_turn=1;
   
   

   
     __GmLeft.uiPulse =60000;   //60400   
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =60000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  
    
 
    
   
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
//   mouseStop(); while(1);
End:
  {  __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;}
}

 
void mouseTurnright_90_v(void)

{   __GmSPID.sRef=200;
//mouseStop();while(1);
   uint8 cL = 0,cR=0;
   __GmLeft.uiPulse =12000;    //9000 7500         
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =12000;           
   __GmRight.uiPulseCtr=0;

   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   
   
   __GmWPID.sRef=-120;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>84000) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;
   

   
      __GucMouseState = __GOAHEAD_45;
  
  // GuiSpeedCtr = __SPEEDUP ;
   __GmLeft.uiPulse = 32000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 32000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  
End: {
    GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}
}


void mouseTurnright_135_v(void)
{  
   int8 cL = 0, cR = 0;
  // __GucMouseState   = __GOAHEAD_45;
    __GmSPID.sRef=140;
   __GmLeft.uiPulse = 17600;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 17600;           
   __GmRight.uiPulseCtr=0;
   __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
  //mouseStop();while(1);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 3) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=200;
   __GmWPID.sRef=-100;
     while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>132000) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;
   __GucMouseState   = __GOAHEAD;
    
    
     __GmLeft.uiPulse =55000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =55000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    
    
    
    
    
    
    
    
   End: 
  {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
  }
}
void mouseTurnright_135_t(void)
{  
 
  
   uint8 cL = 0,cR=0;
   static uint8 i=0;
   i++;
     
    __GmSPID.sRef=180;
    piancha_r = (leftdis - 40)/10;
   if(leftdis > 120){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
   if(usepiancha==0) piancha_r=0;
   
   __GmLeft.uiPulse =32000-piancha_r*2886.67;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =32000-piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;

   GW=0;
   time=0; 
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN  ;
 

   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GucMouseDir     = (GucMouseDir + 3) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=140;
   __GmWPID.sRef=-120;
     while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>132000) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
   __GmWPID.sRef=0; 
  __GmSPID.sRef=200;
   
     __GucMouseState = __GOAHEAD_45;
     
   
   //mouseStop();while(1);
   //GuiSpeedCtr = __SPEEDUP;
   __GmLeft.uiPulse = 22000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 22000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
End:{ __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
  __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;GuiSpeedCtr=__SPEEDUP;
   __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;}
}



extern uint8 xiexianyige;
static uint8 jjjjjj=0;




void mouseGoaheadhui_45 (uint8_t  cNBlock)
{
  // mouseStop();while(1);
    uint8_t cL = 1, cR = 1, cCoor = 1,cB,L_max, R_max,r_over,l_over;
    
    zuihouyige=0;
    cB=cNBlock;
    __GucMouseState   = __GOAHEAD_45;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock *ONEBLOCK_45;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock *ONEBLOCK_45;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
     //__GmRight.cState  = __WAITONESTEP;
     //__GmLeft.cState   = __WAITONESTEP;
   
    GuiSpeedCtr=__SPEEDUP;
    //__GiMaxSpeed=50;
    if(cNBlock > 5)
    {
      __GiMaxSpeed=260;
    }
    else if(cNBlock == 5)
    {
      __GiMaxSpeed=230;
    }
    else if(cNBlock == 4)
    {
      __GiMaxSpeed=230;
    }
    else if(cNBlock == 3)
    {
      __GiMaxSpeed=230;
    }
    else if(cNBlock == 2)
    {
      __GiMaxSpeed=230;
    }
    else  __GiMaxSpeed=130;
    
 
    
  // if(cNBlock > 3){__GiMaxSpeed=sqrt(2*400*((cNBlock-1)*5.7-5.7)+2500);}//a=3.3m/s2
  // else  {__GiMaxSpeed=50;
  //  }
    while (__GmLeft.cState != __MOTORSTOP) 
    {
       
       
      
        if (__GmLeft.uiPulseCtr >= ONEBLOCK_45) 
        {                          
            __GmLeft.uiPulse    -= ONEBLOCK_45;
            __GmLeft.uiPulseCtr -= ONEBLOCK_45;         
            cNBlock--;
            if(cNBlock==0)
              break;
        }

        
            if(cNBlock < 3)
        {
            if(__GmSPID.sFeedBack>220)
            {              
                __GiMinSpeed=150;
                GuiSpeedCtr= __SPEEDDOWN;
            }          
        }
        
        
        if (cNBlock < 2)   //2//45度时，最后一格
        {
  /*  if(__GmSPID.sFeedBack>150)
            {
               __GmSPID.sRef=150;
                 GuiSpeedCtr= __SPEEDDOWN;
            }  
           */
          

          __GmLeft.uiPulse =40000;           
          __GmLeft.uiPulseCtr=0;
          __GmRight.uiPulse =40000; 
          __GmRight.uiPulseCtr=0;   
          while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
          while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
          zuihouyige=1;
          
         }

    }
            
End:   
     
  zuihouyige=0;
 
          
}


void mouseGoahead_L (int8  cNBlock)                                    //连续转弯用
{
    int8 cL = 0, cR = 0, cCoor = 1,cB;
    int16 GuiTpusle_LR_l,GuiTpusle_LR_r;
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    if(cNBlock==1)
    {
        cL = 1;
        cR = 1;
        //GuiTpusle_LR = 0;    //100
        if(GucFangXiang == GucDirTemp)
        {
           //GucYiBaiBa=1;
           GuiTpusle_LR = 5000;//13000
           GuiTpusle_S  = 0;
           if(GucDirTemp==2 ){GuiTpusle_LR_r=0;W_l=0;}//zuojiezuo
            if(GucDirTemp== 6 ){ GuiTpusle_LR_l=0;W_r=0;}//youjieyou
        }
        else
          
        {
           GuiTpusle_LR = 2000;
           GuiTpusle_S  = 0;
          // mouseStop();while(1);
           if(GucDirTemp==2 ){GuiTpusle_LR_r=0;W_l=0;}//zuojieyou
            if(GucDirTemp== 6 ){ GuiTpusle_LR_l=0;W_r=0;}//youjiezuo
           
        }

        __GiMaxSpeed = 150;//120
    }
    else{
        GuiTpusle_LR = 0;
        GuiTpusle_S  = 0;
        __GiMaxSpeed = 150;
    }
    GucFangXiang = GucDirTemp;
    if(((GmcMouse.cX==7)&&(GmcMouse.cY==7))|| 
         ((GmcMouse.cX==8)&&(GmcMouse.cY==8))||
         ((GmcMouse.cX==7)&&(GmcMouse.cY==8))||
           ((GmcMouse.cX==8)&&(GmcMouse.cY==7))){
       cL = 0;
       cR = 0;
       GuiTpusle_LR = 0;
       GuiTpusle_S  = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr = 30000;       //1182(34mm)
        __GmRight.uiPulseCtr = 30000;

    }
    
    cB=cNBlock;
    __GucMouseState   = __GOAHEAD;
   // __GiMaxSpeed      =   25;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock * ONEBLOCK ;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock * ONEBLOCK ;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    
      if(cNBlock > 5)
    {
        __GiMaxSpeed=300;
    }
    else if(cNBlock == 5)
    {
        __GiMaxSpeed=300;
    }
    else if(cNBlock == 4)
    {
        __GiMaxSpeed=200;
    }
    else if(cNBlock == 3)
    {
        __GiMaxSpeed=200;
    }
    else if(cNBlock == 2)
    {
        __GiMaxSpeed=180;
    }
    else;
    
    while (__GmLeft.cState != __MOTORSTOP)
    {
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) 
        {                          /*  判断是否走完一格            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
                cNBlock--;
                if(cNBlock==0)
                   goto End;
                if(cNBlock<cB-1)//给回速一个时间
                  GuiSpeedCtr=__SPEEDUP;
            } else {
                cCoor = 1;
            }
        }
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                     /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if (cNBlock < 2) {
          if(__GmSPID.sFeedBack>150){
              GuiSpeedCtr= 3;
             __GmSPID.sRef=150;
          }
          if (cL) 
          {                                                       /*  是否允许检测左边            */
            if (ucIRCheck[2]<GusDistance_L_Far)             /*  左边有支路，跳出程序        */
            {                 
                __GmRight.uiPulse = __GmRight.uiPulseCtr  + 18000 - GuiTpusle_LR - GuiTpusle_LR_l;    //3094(89mm)
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr   + 18000 - GuiTpusle_LR - GuiTpusle_LR_l;
                while (ucIRCheck[2]<GusDistance_L_Far) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 100) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {                                                    /*  左边有墙时开始允许检测左边  */
                if (ucIRCheck[2]>GusDistance_L_Far) {
                    cL = 1;
                }
            }
         if (cR) 
            {                                                       /*  是否允许检测右边            */
            if (ucIRCheck[3]<GusDistance_R_Far)               /*  右边有支路，跳出程序        */
            {                
                __GmRight.uiPulse  = __GmRight.uiPulseCtr + 22500- GuiTpusle_LR - GuiTpusle_LR_r;
                __GmLeft.uiPulse   = __GmLeft.uiPulseCtr  + 22500- GuiTpusle_LR - GuiTpusle_LR_r;
                while (ucIRCheck[3]<GusDistance_R_Far) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 100) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {
                if (ucIRCheck[3]>GusDistance_R_Far) {                   /*  右边有墙时开始允许检测右边  */
                    cR = 1;
                }
            }
        }
   }
    /*
     *  设定运行任务，让电脑鼠走到支路的中心位置
     */
End:     ;//GuiSpeedCtr= 3;
         //__GmSPID.sRef=23;
}



void mouseStop(void)
{   
  __GmSPID.sRef=0;
  __GmWPID.sRef=0;
  GuiSpeedCtr=5;  
}
/****************************************************************************************************
** Function name:       mouseTurnright
** Descriptions:        右转
** input parameters:    无
** output parameters:   无
** Returned value:      无         按步数转弯
*********************************************************************************************************/
static uint8 after_turnright;

static uint8 aaaaa;

static uint8 lwt1;

float bc_r;


static int sssss=0;
void mouseTurnright(void)
{  
static int ir=0;  

//标定值
 float leftd=left_distance;

  piancha_r = pianchar(leftdis,leftd);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;       
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
  
    
   

   
   __GmSPID.sRef=145;
   __GmWPID.sRef=-85;//-90//-85
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>87400)//81400 //82400 84400 
       {
         break;
       } 
      
   }
   ir++;

      
   __GmWPID.sRef=0;     
  __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   
   __GmLeft.uiPulse =12000- piancha_r*2886.67;      //8000 //9000   9500 8000 7000 6800 6200 5800 5400 5200//4800
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =12000- piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;
   
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   __GucMouseState   = __TURNRIGHT;
   
 //  if(ir==1){ mouseStop(); while(1);} 
 //  if(ir==1){ mouseStop(); while(1);}
   // mouseStop();      while(1);           
     
     
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;

}

static int lllllw=0;
void mouseTurnrightyuandi(void)
{
  __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   GuiSpeedCtr=3;
__GmSPID.sRef=30;
  __GmLeft.uiPulse =8500;           //7700
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =8500;            
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  __GmSPID.sRef=0;
     while(1)
   {
     __GmWPID.sRef=-4; 
       if(jiao>90)      //315000
       {
         break;
       }                        
   }
   
   after_turnright=1;
    __GmWPID.sRef=0; 
   
 
   /* qidianclock=0;
       lllllw=0;
  while(1)
   {

   lllllw++;if(lllllw==3000){lllllw=0;}
     if(lllllw==1)
     {
      if(rightdis<140)
               {
                GsTpusle_T = -0.6*(rightdis - 45);GPIO_ResetBits(GPIOB,GPIO_Pin_12);
                
            
                 
                 __PIDContr();
               }
              else  if(leftdis<120)
               {
                
                  GsTpusle_T = 0.5*(leftdis - 45);__PIDContr();GPIO_SetBits(GPIOB,GPIO_Pin_12);
               }
               else if((rightdis>140)&&(leftdis>120)){GsTpusle_T=0;__PIDContr();GPIO_ResetBits(GPIOB,GPIO_Pin_12);}
               
     
     if(qidianclock==500){qidianclock=0;break;}
     
     }
   
   
   }*/
   
   
 __GmSPID.sRef=50;
  
   __GucMouseState   = __GOAHEAD;
   
   //piancha_r=0;
  // GuiSpeedCtr=3;
   __GmLeft.uiPulse =16000;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =16000;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   //Continuous=1;
  zhixianjuli=0;
  bianmaqijiaodu = bianmaqijiaodu + 1.571;

   __GucMouseState   = __TURNRIGHT;
   
//mouseStop();while(1);
    
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;

  

}


void mouseRT90(void)
{  
 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   
  __GmLeft.uiPulse =9800;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =9800; 
    __GmRight.uiPulseCtr=0;   
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   
   start_speedr=1;//滴答定时器中语句响应
   piancha_r = (leftdis - 66)/10;
   if(leftdis > 120){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
   __GmSPID.sRef=50;
   while(1)
   {
     __GmWPID.sRef=-speed_r[start_flag]-0.6;//给每时刻的线速度角速度赋值，进行弧线转弯-0.6
     if(end_speedr == 1){end_speedr=0;break;} 
   }
   
  
   after_turnright=1;
   
 
   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;
  
   
   lingshiL=ucIRCheck[2];
   lingshiR=ucIRCheck[3];
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulseCtr=0;
   while(1)
  {
   __GmLeft.uiPulse =38000 - piancha_r/0.00052;           //27000//33000
   
   __GmRight.uiPulse =38000 - piancha_r/0.00052;            
   
   if(((__GmRight.uiPulseCtr+200 ) >= __GmRight.uiPulse)&&((__GmLeft.uiPulseCtr+200 ) >= __GmLeft.uiPulse)) 
   {
     break;
   }
    if(lingshiL<ucIRCheck[2]){lingshiL=ucIRCheck[2];}
    if(lingshiR<ucIRCheck[3]){lingshiR=ucIRCheck[3];}
    if((lingshiL>=1300)&&(ucIRCheck[2]<=800)){break;}
    if((lingshiR>=1300)&&(ucIRCheck[3]<=800)){break;;}
            
  }
   

  zhixianjuli=0;
  bianmaqijiaodu = bianmaqijiaodu + 1.571;

   __GucMouseState   = __TURNRIGHT;
   
 
    
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;

  
}
void mouseTurnright_180(void)
{  
   //i++;
   
   
 // mouseStop();while(1);
   int8 cL = 0, cR = 0;
    __GucMouseState   = __GOAHEAD;
    
    
     piancha_r = (leftdis - 40)/10;
   if(leftdis > 120){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
    
    
    
   __GmLeft.uiPulse =20000;    //9000    //14400      
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse= 20000;           
   __GmRight.uiPulseCtr=0;
  __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
  
   GucMouseDir     = (GucMouseDir + 4) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=160;
   __GmWPID.sRef=-100;
   jiao=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>83400)//81400 //82400 84400 
       {
         break;
       } 
      
   }
   
   
    
   __GmWPID.sRef=0;   
  
   __GucMouseState   = __GOAHEAD;
 //  GuiSpeedCtr=3;
   __GmLeft.uiPulse =22000- piancha_r*2886.67;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =22000- piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
       
  
    
     
    
     
  
   __GucMouseState   = __TURNRIGHT; 
   
   
   __GmWPID.sRef=-100;
   W=0;
   GW=0;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>83800)//81400 //82400 84400 
       {
         break;
       } 
      
   }
   
   // mouseStop();while(1);
  //  mouseStop();while(1);

 GoHead45_flag=0;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;   
  __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
    
   __GmLeft.uiPulse =55000;    //9000    //14400      25400
      __GmLeft.uiPulseCtr=0;
      __GmRight.uiPulse =55000;           
      __GmRight.uiPulseCtr=0;
      __GmRight.cState  = __MOTORRUN;
      __GmLeft.cState   = __MOTORRUN;  
      while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
      while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
      //mouseStop();while(1);
      
   //GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
   __rightMotorContr();
   __GmLeft.sSpeed = 0;
   __leftMotorContr();
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   End: 
  {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
  }
}

void mouseTurnright_45(void)

{  
 // j++;
  // mouseStop();
  // while(1);
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;  
   __GucMouseState   = __TURNRIGHT_45; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 1) % 8;                           /*  方向标记                    */
   
   __GmSPID.sRef=100;
   __GmWPID.sRef=-80;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>20000)   
       {
         break;
       } 
      
   }
  mouseStop();   while(1);
   __GmWPID.sRef=0;     
 
   __GucMouseState   = __GOAHEAD;
   __GmLeft.uiPulse =8000;    //4200          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =8000;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  
  //mouseStop();
  //while(1);
  
  
 /* flag_right++;                                                                 //转弯调试
  if(flag_right==2)
  {
    // __delay(9000000);
     mouseStop();  
     while(1);
  }   */ 
     
  

  // __GucMouseState   = __TURNRIGHT;
   GuiSpeedCtr=__SPEEDUP;
    //__GmRight.cState = __MOTORSTOP;       
    //__GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
    
  //  if(j==2)
  //  {
   //   mouseStop();
   //   while(1); 
  //  }
    
}


void mouseTurnright_45_v(void)
{   __GmSPID.sRef=200;  
   int8 cL = 0, cR = 0;
   __GmLeft.uiPulse =23500;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =23500;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   time=0;
   
   //__GucMouseState   = __TURNRIGHT; 
   __GucMouseState   = __TURNRIGHT_45; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 1) % 8;                             /*  方向标记                    */
             
            ///120
   __GmWPID.sRef=-120;//70
   GW=0;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>42000)//81400 //82400 84400 
       {
         break;
       } 
      
   }
 //  mouseStop(); while(1);
 // mouseStop(); while(1);
   __GmSPID.sRef=200; 
    GoHead45_flag=0;
     GoHead45_fx=1;

   __GmWPID.sRef=0;     
   __GucMouseState   = __GOAHEAD;
    __GmLeft.uiPulse =43500;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =43500;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
   __rightMotorContr();
   __GmLeft.sSpeed = 0;
   __leftMotorContr();
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   End: 
   {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
   }
}




void mouseTurnright_45_t(void)
{
  static uint8 i;i++;
  int8 cL = 0;
    
     
    piancha_r = (leftdis - 40)/10;
   if(leftdis > 120){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
   if(usepiancha==0) piancha_r=0;
     
  __GmLeft.uiPulse =6000;//-piancha_r*2886.67;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =6000;///-piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
 
   GW=0;  
   //__GucMouseState   = __TURNRIGHT;
   __GucMouseState   = __TURNRIGHT_45; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 1) % 8;                           /*  方向标记                    */
    __GmSPID.sRef=160;

   __GmWPID.sRef=-100;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>42400)//81400 //82400 84400 
       {
         break;
       } 
      
   }
 // if(i==2){   mouseStop();while(1);   }
   __GucMouseState = __GOAHEAD_45;
  
   __GmWPID.sRef=0; 
   __GmSPID.sRef=220;
   
    __GmLeft.uiPulse =35000;//-piancha_r*2886.67;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =35000;///-piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    //GuiSpeedCtr=__SPEEDUP;
    //__GmRight.cState = __MOTORSTOP;       
    //__GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
End: {//GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;}
}


void mouseTurnleft_45(void)
{
   
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   time=0;
   __GucMouseState   = __TURNLEFT_45;   
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 7) % 8;                             /*  方向标记                    */
             
   __GmSPID.sRef=100;           ///120
   __GmWPID.sRef=80;//70
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>17500*1.5)   
       {
         break;
       } 
      
   }

   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;

   __GmLeft.uiPulse =8000;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =8000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  
//    mouseStop();
//    while(1);
   
  /*  flag_left++;                                                                 //转弯调试
   if(flag_left==2)
   {
      mouseStop();
      while(1);
   }   */
   
   GuiSpeedCtr=__SPEEDUP;

    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}


void mouseTurnleft_45_v(void)
{
   int8 cL = 0, cR = 0;
   static int i=0;
   i++;
 //  mouseStop();while(1);
   __GmSPID.sRef=160;  
 //mouseStop();while(1);
  //  __GucMouseState   = __GOAHEAD_45;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   __GmLeft.uiPulse = 26000;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 26000;           
   __GmRight.uiPulseCtr=0;

   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   time=0;
   
   __GucMouseState   = __TURNLEFT_45;//mouseStop();while(1);
  // __GucMouseState   = __TURNLEFT;
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 7) % 8;                             /*  方向标记                    */
             
         ///120
   __GmSPID.sRef=160;  
   __GmWPID.sRef=100;//70
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>42200)//81400 //82400 84400 
       {
         break;
       } 
      
   }
 
  //  
__GmSPID.sRef=160;
  __GmWPID.sRef=0;     
  
 
  
   __GucMouseState   = __GOAHEAD;
  // GuiSpeedCtr=3;
    
     __GmLeft.uiPulse =45000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =45000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
   __rightMotorContr();
   __GmLeft.sSpeed = 0;
   __leftMotorContr();
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   End: 
   {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
   }
}

void mouseTurnleft_45_t(void)
{
  
    static int i=0;i++;
    int8 zjbl=100;  
    int8 cR = 0;
 
      __GmSPID.sRef=140;
 
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;   
    __GmLeft.uiPulse = 7500 ;   //5000     
    __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 7500 ;           
   __GmRight.uiPulseCtr=0;

   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);  
   GW=0;  
   //__GucMouseState   = __TURNLEFT;
   __GucMouseState   = __TURNLEFT_45;
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 7) % 8;                           /*  方向标记                    */
   
    
   __GmWPID.sRef=80;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>42000)//81400 //82400 84400 
       {
         break;
       } 
      
   }
  // mouseStop();while(1);
   angle_temp=-45;
   after_turn=1;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=240;
   
   
      __GucMouseState = __GOAHEAD_45;
    
   //GuiSpeedCtr = __SPEEDUP ;
   __GmLeft.uiPulse =16000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =16000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
//   if(i==1){
//   mouseStop();while(1);
//   }
    //GuiSpeedCtr=__SPEEDUP;
    //__GmRight.cState = __MOTORSTOP;       
    //__GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
End:
  
  
   __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
  
  
  ;
  // if(i==1){
//   mouseStop();while(1);
   
}



/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        左转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/

 float bc_l;

static uint8 lwt;
void mouseTurnleft(void)
{
static int il=0;

  float rightd = right_distance;//标定值
  
  piancha_l = pianchal(rightdis,rightd);
 
   time=0;   
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
             
   __GmSPID.sRef=145;           ///120//150
   __GmWPID.sRef=95;//70//90
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>84000) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
   il++;
    
   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
     __GmLeft.uiPulse =16500- piancha_l*2886.67;     //10000 //5500 5000   
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =16500- piancha_l*2886.67;            
   __GmRight.uiPulseCtr=0;

             
              
 
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   

   __GucMouseState   = __TURNLEFT;
   
     
   
   
   
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;

   
}

void mouseTurnleftyuandi(void)
{
   time=0;   
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
             

   __GmWPID.sRef=80;//70
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>86000)   
       {
         break;
       } 
      
   }
   __GmWPID.sRef=0; 
  //__GmWPID.sRef=0; 
    __GucMouseState   = __GOAHEAD;
  __GmSPID.sRef=80;
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =2000;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =2000;           
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    

   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;

}

void mouseLT90(void)
{
  static int j=0;
   j++;
   
    __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
  
    __GmLeft.uiPulse =10700;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =10700; 
    __GmRight.uiPulseCtr=0;   
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
 
   
   
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                             /*  方向标记                    */
             
   start_speedl=1;//滴答定时器中语句响应
   piancha_l = (rightdis - 66)/10;
   if(rightdis>120){piancha_l = 0;}//左转右边没墙
   if(piancha_l > 2){piancha_l = 2;}
    if(piancha_l < -2){piancha_l = -2;}
   __GmSPID.sRef=50;
   while(1)
   {
     __GmWPID.sRef=speed_r[start_flag1]+0.7;//给每时刻的线速度角速度赋值，进行弧线转弯+0.7
     if(end_speedl == 1){end_speedl=0;break;} 
   }
 
   after_turnleft=1;

   
  //mouseStop(); while(1);
   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;
 //piancha_l=0;
   
   GuiSpeedCtr=3;
   //mouseStop();while(1);
   
   lingshiL=ucIRCheck[2];
   lingshiR=ucIRCheck[3];
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulseCtr=0;
   while(1)
  {
   __GmLeft.uiPulse =27000 - piancha_l/0.00052;           //7700
   
   __GmRight.uiPulse =27000 - piancha_l/0.00052;            
   
   if(((__GmRight.uiPulseCtr+200 ) >= __GmRight.uiPulse)&&((__GmLeft.uiPulseCtr+200 ) >= __GmLeft.uiPulse)) 
   {
     break;
   }
    if(lingshiL<ucIRCheck[2]){lingshiL=ucIRCheck[2];}
    if(lingshiR<ucIRCheck[3]){lingshiR=ucIRCheck[3];}
    if((lingshiL>=1300)&&(ucIRCheck[2]<=350)){break;}
    if((lingshiR>=1300)&&(ucIRCheck[3]<=350)){break;;}
            
  }
  //mouseStop();while(1);
 

   zhixianjuli=0;
   bianmaqijiaodu = bianmaqijiaodu - 1.571;
  
   __GucMouseState   = __TURNLEFT;
 
   
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
   
 
   
}

 

void mouseTurnleft_180(void)
{
 // mouseStop();
//  while(1);
   //j++;
  
   __GucMouseState   = __GOAHEAD;
   int8 cL = 0, cR = 0;
    piancha_l = (rightdis - 55)/10;
   if(rightdis>125){piancha_l = 0;}//左转右边没墙
   if(piancha_l > 2){piancha_l = 2;}
    if(piancha_l < -2){piancha_l = -2;}
   __GmLeft.uiPulse =13000;    //9000    //14400   17000
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse= 13000;           
   __GmRight.uiPulseCtr=0;
    __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   time=0;   
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 4) % 8;                             /*  方向标记                    */
             
   __GmSPID.sRef=200;           ///120
   __GmWPID.sRef=120;//70
   GW=0;
   jiao=0;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>81400)//81400 //82400 84400 
       {
         break;
       }
      
   }
   
   __GucMouseState   = __GOAHEAD;
   //mouseStop();while(1);  
      __GmWPID.sRef=0; 
    __GmLeft.uiPulse =18000- piancha_l*2886.67;            //7700
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =18000- piancha_l*2886.67;             
   __GmRight.uiPulseCtr=0;
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   
    
    __GucMouseState   = __TURNLEFT; 
    GW=0;
   W=0;
      __GmWPID.sRef=120;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>77400)//81400 //82400 84400 
       {
         break;
       } 
      
   }
  // mouseStop();while(1);
   GoHead45_flag=0;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;   
  __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
 
    
      __GmLeft.uiPulse =35000;    //9000    //14400      25400
      __GmLeft.uiPulseCtr=0;
      __GmRight.uiPulse =35000;           
      __GmRight.uiPulseCtr=0;
      __GmRight.cState  = __MOTORRUN;
      __GmLeft.cState   = __MOTORRUN;  
      while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
      while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
      //mouseStop();while(1);
      
   
   //GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
   __rightMotorContr();
   __GmLeft.sSpeed = 0;
   __leftMotorContr();
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   End: 
  {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
  }
}

/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        左转7
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnright_C(void)
{
  static int q=0;
  
   float leftd=74;

  piancha_r = pianchar(leftdis,leftd);
  q++;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
 
   __GmSPID.sRef=145;
   __GmWPID.sRef=-125;
 
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>80400)//81400 //82400 84400 
       {
         break;
       } 
      
   }
   
   __GmWPID.sRef=0;     
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =1500- piancha_r*2886.67;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =1500- piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
 
   __GucMouseState   = __TURNRIGHT;
 
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
  //mouseStop();
    //while(1);
    //if(i==)
    //{
      //mouseStop();
      //while(1);
    //}
   
}
/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        左转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnleft_C(void)
{static int j=0;

float rightd = 70;//标定值
  
  piancha_l = pianchal(rightdis,rightd);
  
   j++;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   time=0;  
   
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                             /*  方向标记                    */
             
   __GmSPID.sRef=145;           ///120
   __GmWPID.sRef=95;//70
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>81900)//81400 //82400 84400 
       {
         break;
       } 
      
   }
  
 //  if(j==2){mouseStop();while(1);}
   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =1000- piancha_l*286.67;  ;           //7700
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =1000- piancha_l*286.67;  ;            
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
  // mouseStop();
    //while(1);  
   __GucMouseState   = __TURNLEFT;
   
//   mouseStop();
//    while(1);  
   
   
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
    //if(j==5)
    //{
      //mouseStop();
      //while(1); 
    //}
   
}
void mouseTurnright_Y1(void)
{  
   __GmLeft.uiPulse =6000;            // 135*32*1024/(12*25*3.14)  R_r=14mm  R_l=14+72=86 ;4697
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =6000;           // 22*32*1024/(12*25*3.14);
   __GmRight.uiPulseCtr=0;
   __GmSPID.sRef=0;
   __GmWPID.sRef=-20;
   GW=0;
   __GucMouseState   = __TURNRIGHTY; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>90000)   
       {
         break;
       } 
      // if(W>70000)
      //   __GmWPID.sRef=10;
   }
   __mazeInfDebug();

   __GmWPID.sRef=0;
   __GmSPID.sRef=50;

   GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
  // __delay(100000);
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
}




/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        左转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnleft_Y1(void)
{  
  // __delay(2000000);
   __GmLeft.uiPulse =6000;            // 22*32*1024/(12*25*3.14)  R_l=14mm  R_r=14+72=86
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =6000;           // 135*32*1024/(12*25*3.14)
   __GmRight.uiPulseCtr=0;
   
   __GmWPID.sRef=24;
   GW=0;
   __GucMouseState   = __TURNLEFTY; 
 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>90000)   
       {
         break;
       } 
      // if(W>70000)
      //   __GmWPID.sRef=10;
   }
   __mazeInfDebug();
   __GmWPID.sRef=0;
   __GmSPID.sRef=25;
  
   GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
   //__delay(100000);
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
}


void mouseTurnright_Y(void)
{  
GW=0;
   time=0;   
   
    /*   __GucMouseState   = __GOAHEAD;
       
  __GmLeft.uiPulse =15000;          //7700//15500
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =15000;            
   __GmRight.uiPulseCtr=0;

   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
 
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);*/
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=0;
   __GmWPID.sRef=-80;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>92600)   
       {
         break;
       } 
      
   }
    
   __GmWPID.sRef=0;  
   __GmSPID.sRef=120;
   __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =27000;          //7700//15500
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =27000;            
   __GmRight.uiPulseCtr=0;

              
 
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
//     mouseStop();while(1);
   
      __GucMouseState   = __TURNRIGHT;
   //   mouseStop();while(1);
  GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}
/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        左转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnleft_Y(void)
{
  time=0;   
  
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
             

   __GmWPID.sRef=80;//70
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>90000)   
       {
         break;
       } 
      
   }
   
  
   __GmWPID.sRef=0; 
   __GmSPID.sRef=120;
   __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =25000;          //7700//15500
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =25000;            
   __GmRight.uiPulseCtr=0;

              
 
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   

   __GucMouseState   = __TURNLEFT;
   // mouseStop();while(1);
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}




void onestep(void)
{
     __GiMaxSpeed = 50;
     __GucMouseState   = __GOAHEAD;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
     __GmRight.uiPulse =6000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse =6000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
//     mouseStop();
//     while(1);
   
}

void onestep1(void)
{
     __GmRight.uiPulse =12000;  // 75000          
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse =12000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
}
void onestep2(void)
{
     __GmRight.uiPulse =15000;          
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse =15000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
}


void onestep3(void)
{
     __GmRight.uiPulse =20000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse =20000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
}
/*********************************************************************************************************
** Function name:       mouseTurnback
** Descriptions:        根据前方近距，旋转180度
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnback1(void)
{ 
   __GmSPID.sRef=0;  
    GW=0;
    time=0;
     __GucMouseState   = __TURNBACK;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;                                  
    while(1)
   {
      if(time>604)
        break;
   }

   /* __GmSPID.sRef=-80; 
     __GmRight.uiPulse =16000;          
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse =16000;
     __GmLeft.uiPulseCtr=0;  
     while ((__GmRight.uiPulseCtr+100 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+100 ) <= __GmLeft.uiPulse); -*/      
    __GucMouseState   = __GOAHEAD ;
    mouseStop();
    __delay(1500000);
     GuiSpeedCtr=__SPEEDUP;
         __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}


void mouseTurnback_Y3(void)
{ 
  
  if(GucFrontNear)
  {
      __GmSPID.sRef=120;
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));
       __GucMouseState   = __GOAHEAD ;
     __GmRight.uiPulse =1000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 1000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);    
    
  }
      backflag=0;
     if((ucIRCheck[2]>GusDistance_L_Near))   //偏左
     {
       backflag=1;
     }
    GucFrontNear=0;
 
    __GmSPID.sRef=0; 
    mouseStop();
    GW=0;
    time=0;
    
    __GucMouseState   = __TURNBACK;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;         
   
       
    while(1)
   {
      if(time>725)//597
        break;
   }
    __GmWPID.sRef=0;
     mouseStop();
    __GucMouseState   = __GOBACK ;
    __GmSPID.sRef=-40;
     __GmRight.uiPulse =15000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 15000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);    
  __GmSPID.sRef=0;
     mouseStop();
    __GucMouseState   = __GOAHEAD ;
    __delay(2000000);
   
    GuiSpeedCtr=__SPEEDUP;
    
   /* __GucMouseState   = __GOAHEAD ;
   // mouseStop();
    //while(1);
    __GmRight.uiPulse =28000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 28000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); */    
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;

    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;//3000
    __GmLeft.uiPulseCtr = 0;//3000
     
}








void mouseTurnback(void)
{ 
if(GucFrontNear)
  {
      __GmSPID.sRef=50;//120
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));
     __GmRight.uiPulse =1000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 1000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);      
  }
  GucFrontNear=0;
  backflag=1;
   __GmSPID.sRef=0; 
 //  mouseStop();
    GW=0;
    time=0;
    __GucMouseState   = __TURNBACK;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;    
        __GmWPID.sRef=90;//80
 
   while(1)
   {
       if(GW>290000)  
       {
         break;
       }                        
   }
// mouseStop();while(1);
    __GmWPID.sRef=0;

   __delay(5000000);
   __GucMouseState   = __GOAHEAD ;

    GuiSpeedCtr=__SPEEDUP;
    __GmSPID.sRef=150;//120
  
    __GmRight.uiPulse =23000;    //23000         
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 23000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);  

    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;

    
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 23000;
    __GmLeft.uiPulseCtr = 23000;
}

 
void mouseTurnbackqidian(void)
{ 
  
 if(GucFrontNear)
  {
      __GmSPID.sRef=150;//120
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));
     __GmRight.uiPulse =1000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 1000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);      
  }
  GucFrontNear=0;
  backflag=1;
   __GmSPID.sRef=0; 
   mouseStop();
    GW=0;
    time=0;
    __GucMouseState   = __TURNBACK;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 4;    
        __GmWPID.sRef=90;//80
 
   while(1)
   {
       if(GW>300000)  
       {
         break;
       }                        
   }

    __GmWPID.sRef=0;

   __delay(5000000);
   __GucMouseState   = __GOAHEAD ;

    GuiSpeedCtr=__SPEEDUP;
    __GmSPID.sRef=150;//120
  
    __GmRight.uiPulse =3000;    //23000         
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 3000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);  

    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;

    
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 3000;
    __GmLeft.uiPulseCtr = 3000;
}




void mouseTurnback_js(void)
{ 

  if(GucFrontNear)
  {
      __GmSPID.sRef=100;
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));
     __GmRight.uiPulse =1000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 1000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);      
  }
  GucFrontNear=0;
  backflag=1;
   __GmSPID.sRef=0; 
   mouseStop();
    GW=0;
    time=0;
    __GucMouseState   = __TURNBACK;

    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 4;    
        __GmWPID.sRef=80;
  
   while(1)
   {
       if(GW>290000)      //315000
       {
         break;
       }                        
   }
   
   __GmWPID.sRef = 0;                           
    mouseStop();
    __GucMouseState = __GOBACK;
    __GmSPID.sRef = -40;
    __GmRight.uiPulse = 12000;                  
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = 12000;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + 200) <= __GmRight.uiPulse);    
    while ((__GmLeft.uiPulseCtr + 200)  <= __GmLeft.uiPulse);    
  
    mouseStop();
    __GucMouseState = __GOAHEAD;
    __delay(5000000);
    
    GuiSpeedCtr = __SPEEDUP;
    __GmSPID.sRef = 100;                         
    __GucMouseState = __GOAHEAD;
    __GmRight.uiPulse = 3000;   //   //9000            
    __GmRight.uiPulseCtr = 0;  
    __GmLeft.uiPulse = 3000;    //9000                  
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + 200 ) <= __GmRight.uiPulse);    
    while ((__GmLeft.uiPulseCtr + 200 ) <= __GmLeft.uiPulse);    
    
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}


void mouseTurnbackqi(void)
{ 
  if(GucFrontNear)
  {
      __GmSPID.sRef=120;
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));
     __GmRight.uiPulse =1000; 
     
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 1000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);      
  }
  GucFrontNear=0;
  backflag=1;
   __GmSPID.sRef=0; 
 //  mouseStop();
    GW=0;
    time=0;
    __GucMouseState   = __TURNBACK;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;    
        __GmWPID.sRef=80;
  
   while(1)
   {
       if(GW>280000)  
       {
         break;
       }                        
   }
   __GmWPID.sRef = 0;                             
    mouseStop();
    __GucMouseState = __GOBACK;
    __GmSPID.sRef = -40;
    __GmRight.uiPulse = 12000;     //17000              
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = 12000;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + 200) <= __GmRight.uiPulse);    
    while ((__GmLeft.uiPulseCtr + 200)  <= __GmLeft.uiPulse);    
  
    mouseStop();  
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;

    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;               
    __GmLeft.uiPulseCtr = 0;               


}



void mouseTurnback_H(void)
{ 
  backflag=1;
   __GmSPID.sRef=0; 
   mouseStop();
    GW=0;
    time=0;
    __GucMouseState   = __TURNBACK;

    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;    
        __GmWPID.sRef=80;
  
   while(1)
   {
       if(GW>294000)      //315000
       {
         break;
       }                        
   }
   
   __GmWPID.sRef = 0;                           
    mouseStop();//while(1);
   // __GucMouseState = __GOBACK;
    
    __GmSPID.sRef = -20;
    __GmRight.uiPulse = 1000; //12000                 
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = 1000;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + 200) <= __GmRight.uiPulse);    
    while ((__GmLeft.uiPulseCtr + 200)  <= __GmLeft.uiPulse);    
  
  //  mouseStop();while(1);
   // __GucMouseState = __GOAHEAD;
    __delay(5000000);
    
   // GuiSpeedCtr = __SPEEDUP;
    __GmSPID.sRef = 100;                         
    __GucMouseState = __GOAHEAD;
    __GmRight.uiPulse = 3000;   //   //9000            
    __GmRight.uiPulseCtr = 0;  
    __GmLeft.uiPulse = 3000;    //9000                  
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + 200 ) <= __GmRight.uiPulse);    
    while ((__GmLeft.uiPulseCtr + 200 ) <= __GmLeft.uiPulse);    
    
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}
static float w = 0;

static float w1 = 0;
static u16 lala=1;
static float gyro_0=0;
 
void voltageDetect(void)
{
  u16 w;
  if(Angle_TLY_Average>=voltageDetectRef)
  {
     w= Angle_TLY_Average - voltageDetectRef;
  }
  else
  {
     w= voltageDetectRef - Angle_TLY_Average;
  }
  GW=GW+w;
}


/*********************************************************************************************************
** Function name:       mazeSearch
** Descriptions:        前进N格
** input parameters:    iNblock: 前进的格数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
static int16   GuiTpusle_LR_l                     = 0;
static int16   GuiTpusle_LR_r                      = 0; 
 
 
static uint8 sjjj;
void mazeSearch(void)                  
{
    int8 cL = 0, cR = 0, cCoor = 1;
    if (__GmLeft.cState)
    {
        cCoor = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr =40000;
        __GmRight.uiPulseCtr =40000;
        cL = 1;
        cR = 1;
        if(((__GucDistance[__FRONTR]!=0)&&(__GucDistance[__FRONTL]!=0))||((__GucDistance[ __LEFT] & 0x01) == 0)||((__GucDistance[__RIGHT] & 0x01) == 0))
        {
          if((__GucDistance[__FRONTR]!=0)&&(__GucDistance[__FRONTL]!=0))
          {
          GuiTpusle_LR =16000;
            
           
                   
            if((__GucMouseState==__TURNRIGHT)&&((__GucDistance[ __LEFT] & 0x01) == 0)){GuiTpusle_LR_l=500;W_l=0;}//LR 
            if((__GucMouseState==__TURNRIGHT)&&((__GucDistance[__RIGHT] & 0x01) == 0)){GuiTpusle_LR_r=0;W_r=-0;}
              if((__GucMouseState==__TURNLEFT)&&((__GucDistance[ __LEFT] & 0x01) == 0)){GuiTpusle_LR_l=0;W_l=0;}//左接左7200
              if((__GucMouseState==__TURNLEFT)&&((__GucDistance[__RIGHT] & 0x01) == 0)){GuiTpusle_LR_r=-4000;W_r=0;}//1700 700//-1200  
            
             
          }
          else
            GuiTpusle_LR =12000;
        }
        
        else{
          GuiTpusle_LR =0; 
        }
    }
    
    
    else{
      GuiTpusle_LR =0;
    }
    __GucMouseState   = __GOAHEAD;
    __GiMaxSpeed      =   SEARCHSPEED;       
    __GmRight.uiPulse =   MAZETYPE * ONEBLOCK;
    __GmLeft.uiPulse  =   MAZETYPE * ONEBLOCK;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
     GuiSpeedCtr=__SPEEDUP;
    while (__GmLeft.cState != __MOTORSTOP) 
    {
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK)
        {                          /*  判断是否走完一格*/
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) 
            {
                if(((__GucDistance[__FRONTR]!=0)&&(__GucDistance[__FRONTL]!=0))&&(ucIRCheck[2]>GusDistance_L_Far)&&(ucIRCheck[3]>GusDistance_R_Far))//0x01
              {          
               GucFrontNear=1;
               
                goto End;
              }
              __mouseCoorUpdate();                                    /*  更新坐标                    */
            } 
            else 
            {
                cCoor = 1;
            }
        }
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        
        if (cL) {                                                       /*  是否允许检测左边            */
            if  ((__GucDistance[__LEFT]  & 0x01)==0)
            {                 /*  左边有支路，跳出程序        */
            
                __GmRight.uiPulse =  __GmRight.uiPulseCtr + 17800 - GuiTpusle_LR- GuiTpusle_LR_l; //10800 //7500//5000    //1500
                __GmLeft.uiPulse  =  __GmLeft.uiPulseCtr  + 17800 - GuiTpusle_LR- GuiTpusle_LR_l;
                while ((__GucDistance[__LEFT]  & 0x01)==0)
                {
                 
                    if ((__GmLeft.uiPulseCtr + 400) > __GmLeft.uiPulse) 
                    {
                        GucFrontNear=0;
                        goto End;
                    }
                }
                __GmRight.uiPulse = MAZETYPE * ONEBLOCK;
                __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
                GuiSpeedCtr=__SPEEDUP;
            }
        } else {                                                        /*  左边有墙时开始允许检测左边  */
            if (ucIRCheck[2]>GusDistance_L_Far) {
                cL = 1; 
               
            }
        }
        if (cR) {                                                       /*  是否允许检测右边            */
            if ((__GucDistance[__RIGHT]  & 0x01)==0){//mouseStop();while(1);               /*  右边有支路，跳出程序        */
            
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 17800 - GuiTpusle_LR- GuiTpusle_LR_r; //6500 //5000   //3300
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 17800 - GuiTpusle_LR- GuiTpusle_LR_r; 
                
               
                while ((__GucDistance[__RIGHT]  & 0x01)==0) {
                 
                    if ((__GmLeft.uiPulseCtr + 400) > __GmLeft.uiPulse)
                    {
                        GucFrontNear=0;
                        goto End;
                        
                    }
                }
                __GmRight.uiPulse = MAZETYPE * ONEBLOCK;
                __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
                GuiSpeedCtr=__SPEEDUP;
            }
        } else {
            if (ucIRCheck[3]>GusDistance_R_Far)
            {
                cR = 1;
            }
        }
    }
    
End:   
           
          
          __mouseCoorUpdate();                                            /*  更新坐标                    */
        

    
}


/*********************************************************************************************************
** Function name:       __mouseCoorUpdate
** Descriptions:        根据当前方向更新坐标值
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __mouseCoorUpdate (void)
{

    switch (GucMouseDir) {

    case 0:
        GmcMouse.cY++;
        break;

    case 2:
        GmcMouse.cX++;
      
        break;

    case 4:
        GmcMouse.cY--;
        break;

    case 6:
        GmcMouse.cX--;
      
        break;

    default:
        break;
    }
    __wallCheck();
    __mazeInfDebug ();
   
  // if(map==1)
  // {
  //   map=0;
     //uart_sendchar(GmcMouse.cX,GmcMouse.cY);
    // Sendmap();
  // }
    
  
    
}



void __mazeInfDebug (void)
{
    /*
     *  显示方向
     */
    switch (GucMouseDir) {
        
    case 0:
        zlg7289Download(2, 3, 0, 0x47);                                /*  向前，用F表示               */
        break;
        
    case 2:
        zlg7289Download(2, 3, 0, 0x77);                                /*  向右，用R表示               */
        break;
        
    case 4:
        zlg7289Download(2, 3, 0, 0x1f);                                /*  向后，用b表示               */
        break;
        
    case 6:
        zlg7289Download(2, 3, 0, 0x0e);                                /*  向左，用L表示               */
        break;
        
    default :
        zlg7289Download(2, 3, 0, 0x4f);                                /*  错误，用E表示               */
        break;
    }
    /*
     *  显示坐标
     */
    zlg7289Download(1, 0, 0, GmcMouse.cX / 10);
    zlg7289Download(1, 1, 0, GmcMouse.cX % 10);
    zlg7289Download(1, 6, 0, GmcMouse.cY / 10);
    zlg7289Download(1, 7, 0, GmcMouse.cY % 10);
}

/*********************************************************************************************************
** Function name:       __wallCheck
** Descriptions:        根据传感器检测结果判断是否存在墙壁
** input parameters:    无
** output parameters:   无
** Returned value:      cValue: 低三位从左到右一次代表左前右。1为有墙，0为没墙。
*********************************************************************************************************/
void __wallCheck (void)
{
    uint8 ucMap = 0;
    uint8 uctemp = 0;
    ucMap |= MOUSEWAY_B;
    
    if (ucIRCheck[2]>GusDistance_L_Far)//左边有挡板，但是距离较远
    {
        ucMap &= ~MOUSEWAY_L;          //相对方向的左边有墙，并却转换成绝对坐标
        uctemp |= 0x06;
    }
    else 
    {
        ucMap |=MOUSEWAY_L;         //相对方向的左边无墙，并却转换成绝对坐标   
    }
    
    if ((ucIRCheck[0]>GusDistance_FL_Far)&&(ucIRCheck[1]>GusDistance_FR_Far)) 
    {
        ucMap &= ~MOUSEWAY_F;
        uctemp |= 0x40;
        //mouseStop();
           //while(1);
    }
    else 
    {
        ucMap |=  MOUSEWAY_F;
    }
    if (ucIRCheck[3]>GusDistance_R_Far) 
    {
        ucMap &= ~MOUSEWAY_R;
        uctemp |= 0x30;
    }else {
        ucMap |=  MOUSEWAY_R;
    }
    GucMapBlock0[GmcMouse.cX][GmcMouse.cY]=ucMap;
    GucMapBlock[GmcMouse.cX][GmcMouse.cY]=ucMap;
    GucMapBlock1[GmcMouse.cX][GmcMouse.cY]=ucMap;
    if(GmcMouse.cY<(MAZETYPE-1))
        {GucMapBlock1[GmcMouse.cX][GmcMouse.cY+1] &= ~(((~ucMap)&0x01)*4);}       /*将该坐标周围坐标墙壁资料更改  注：洪水用*/
       if(GmcMouse.cX<(MAZETYPE-1))
        { GucMapBlock1[GmcMouse.cX+1][GmcMouse.cY]&= ~(((~ucMap)&0x02)*4);}
         if(GmcMouse.cY>0)
         {GucMapBlock1[GmcMouse.cX][GmcMouse.cY-1]&= ~(((~ucMap)&0x04)/4);}
        if(GmcMouse.cX>0)
         {GucMapBlock1[GmcMouse.cX-1][GmcMouse.cY]&= ~(((~ucMap)&0x08)/4);}
              
      if(GmcMouse.cY<(MAZETYPE-1))
         {GucMapBlock[GmcMouse.cX][GmcMouse.cY+1] |=    ((ucMap&0x01)*4);}        /*将该坐标周围坐标墙壁资料更改  注：在初始为有墙时管用*/
      if(GmcMouse.cX<(MAZETYPE-1))
         { GucMapBlock[GmcMouse.cX+1][GmcMouse.cY]|=  ((ucMap&0x02)*4);}
        if(GmcMouse.cY>0)
         {GucMapBlock[GmcMouse.cX][GmcMouse.cY-1]|=  ((ucMap&0x04)/4);}
        if(GmcMouse.cX>0)
          {GucMapBlock[GmcMouse.cX-1][GmcMouse.cY]|=  ((ucMap&0x08)/4);}
    
    zlg7289Download(2, 2, 0, uctemp);
 
}

uint8 DenggaoCheck (void)//搜索中制作等高图stop，原地转弯判断
{
  if((__GucDistance[__FRONT])&&(__GucDistance[ __LEFT] & 0x01)&&(__GucDistance[__RIGHT] & 0x01)){
        return(true);
    }else {
        return(false);
    }
}
uint8 PulseCtrCheck (void)////搜索中制作等高图stop，已走步数判断
{
  if(__GmLeft.uiPulseCtr  > (__GmLeft.uiPulse+15000)){
        return(true);
    }else {
        return(false);
    }
}

