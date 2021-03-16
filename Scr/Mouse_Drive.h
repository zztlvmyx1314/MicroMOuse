/****************************************Copyright (c)****************************************************
**                                �������ΰҵ�Ƽ����޹�˾
**                                     
**                                 http://www.qcmcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           Mouse.Drive.h
** Last modified Date:  
** Last Version: 
** Description:         �ײ���������ͷ�ļ�
** 
**--------------------------------------------------------------------------------------------------------
** Created By: 
** Created date: 
** Version: 
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**
*********************************************************************************************************/


#ifndef __Mouse_Drive_h
#define __Mouse_Drive_h


/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "stm32f10x.h"
#include "Zlg7289.h"
#include "Type.h"
#include "Micromouse.h"
#include "Mouse_Config.h"
#include "stmflash.h"
/*********************************************************************************************************
  PA�˿ڶ���
*********************************************************************************************************/                                
#define __D2_1                GPIO_Pin_0                                   /*   ��� M2 PWMH      */
#define __D2_2                GPIO_Pin_1                                   /*   ��� M2 PWML      */
#define __D1_1                GPIO_Pin_2                                   /*   ��� M1 PWMH      */
#define __D1_2                GPIO_Pin_3                                   /*   ��� M1 PWML      */
#define __KEY                 GPIO_Pin_11                                  /* KEY�����ӵĶ˿� */
/*********************************************************************************************************
  PB�˿ڶ���

*//*********************************************************************************************************
  PC�˿ڶ���
*********************************************************************************************************/
#define __FRONT_R             GPIO_Pin_0                                  /*  ǰ���Ҵ�����������ź�        */
#define __FRONTSIDE_R         GPIO_Pin_2                                  /*  45�Ҵ�����������ź�      */
#define __RIGHTSIDE           GPIO_Pin_1                                  /*  �ҷ�������������ź�        */
#define __START               GPIO_Pin_10                                  /* START�����ӵĶ˿� */
/*********************************************************************************************************
  PD�˿ڶ���
*********************************************************************************************************/

/*********************************************************************************************************
  PE�˿ڶ���
*********************************************************************************************************/
#define __DIR1                GPIO_Pin_2                                   /*  ������ ��� M1 A��      */
#define __DIR2                GPIO_Pin_3                                   /*  ������ ��� M2 A��      */

#define __FRONT_L             GPIO_Pin_15                                  /*  ǰ���󴫸���������ź�        */
#define __FRONTSIDE_L         GPIO_Pin_13                                  /*  45�󴫸���������ź�      */
#define __LEFTSIDE            GPIO_Pin_14                                  /*  �󷽴�����������ź�        */
/*********************************************************************************************************
  �����궨��--������
*********************************************************************************************************/
#define __LEFT              0                                           /*  �󷽴�����                  */
#define __FRONTL            1                                           /*  ��ǰ��������                */
#define __FRONT             2                                           /*  ǰ��������                  */
#define __FRONTR            3                                           /*  ��ǰ��������                */
#define __RIGHT             4                                           /*  �ҷ�������                  */


/*********************************************************************************************************
  �����궨��--������״̬
*********************************************************************************************************/
#define __STOP              0                                           /*  ������ֹͣ                  */
#define __GOAHEAD           1                                           /*  ������ǰ��                  */
#define __GOAHEAD_45        2                                           /*  ������45�Ⱥ�ǰ��            */
#define __TURNLEFT          3                                           /*  ����������ת                */
#define __TURNRIGHT         4                                           /*  ����������ת                */
#define __TURNBACK          5                                           /*  ���������ת                */
#define __TURNLEFTY         7                                           /*  ����������ת                */
#define __TURNRIGHTY        8                                           /*  ����������ת                */
#define __GOBACK            9
#define __TURNRIGHT_45      10                                         /*  ����������ת */
#define __TURNLEFT_45       11 
#define __RUN45       12 
#define __RUN90VL       13 
#define __RUN90VR       14 
#define __AFTER          15
#define __SPURT45  16
/*********************************************************************************************************
  �����궨��--����Ӽ��ٶ�
*********************************************************************************************************/
#define __SPEEDUP         0                                           /*  �������                    */
#define __SPEEDDOWN       1                                           /*  �������                */

/*********************************************************************************************************
  �����궨��--���״̬
*********************************************************************************************************/
#define __MOTORSTOP         0                                           /*  ���ֹͣ                    */
#define __WAITONESTEP       1                                           /*  �����ͣһ��                */
#define __MOTORRUN          2                                           /*  �������                    */


/*********************************************************************************************************
  �����궨��--������з���
*********************************************************************************************************/
#define __MOTORGOAHEAD      0                                           /*  ���ǰ��                    */
#define __MOTORGOBACK       1                                           /*  �������                    */
#define __MOTORGOSTOP       2                                           /*  ��������ƶ�                */

/*********************************************************************************************************
  �����궨��--PID
*********************************************************************************************************/
#define __KP 20 //80  //���� 30
#define __KI 0.1    //���� 0.01
#define __KD 0           //΢��

#define U_MAX 1400       //���ص����ֵ,��pwm������ֵ 
#define U_MIN 0 
#define error_IMAX 10     //�����޷� 
#define Deadband 3   //�ٶ�PID������������Χ
#define __VmaxSpeed 400
/*********************************************************************************************************
  �ṹ�嶨��
*********************************************************************************************************/
struct __motor {
    int8    cState;                                                     /*  �������״̬                */
    int8    cDir;                                                       /*  ������з���                */
    int8    cRealDir;                                                   /*  ������з���                */
    uint32  uiPulse;                                                    /*  �����Ҫ���е�����          */
    uint32  uiPulseCtr;                                                 /*  ��������е�����            */
    int16   sSpeed;                                                    /*  ��ǰռ�ձ�                    */
};
typedef struct __motor __MOTOR;

struct __pid       //���������������� 
{ 
    //uint16 usRef;      //�ٶ�PID���ٶ��趨ֵ 
    int16 usFeedBack;  //�ٶ�PID���ٶȷ���ֵ//int16zzzzzzzzzzzzzzzzz
    uint16 usEncoder_new; //������
    uint16 usEncoder_last; //������
    
    float sRef;
    float sFeedBack;//int16zzzzzzzzzzzzzzzz
    float sPreError;  //�ٶ�PID��ǰһ�Σ��ٶ����,,vi_Ref - vi_FeedBack 
    float sPreDerror; //�ٶ�PID��ǰһ�Σ��ٶ����֮�d_error-PreDerror; 
  
    fp32 fKp;      //�ٶ�PID��Ka = Kp 
    fp32 fKi;      //�ٶ�PID��Kb = Kp * ( T / Ti ) 
    fp32 fKd;      //�ٶ�PID�� 
       
    int16 iPreU;    //����������ֵ      
};
typedef struct __pid __PID;


/*********************************************************************************************************
  �����궨��
*********************************************************************************************************/
void voltageDetect(void);
void objectGoTo_CC (int8 cXdst, int8 cYdst);
uint8 keyCheck(void);
void mouseInit(void);
void mazeSearch(void);                                                  /*  ǰ��N��                     */
void mouseTurnleft(void);                                               /*  ����ת90��                  */
void mouseTurnright(void);                                              /*  ����ת90��                  */
void mouseTurnback(void);                                               /*  ���ת                      */
void mouseTurnback_H(void);
void sensorDebug(void);                                                 /*  ����������                  */
void voltageDetect(void);
void voltageDetect1(void);
//void mouseGoaheadhui(int8  cNBlock);                                       /*  ǰ��N��                     */
//void mouseGoahead_Llow (int8  cNBlock);
void __UART0Init (void);
void SendChar(uint8 dat);
void SendStr(uint8 *s);
void mouseTurnback_Y(void);
void mouseTurnback_Y3(void);
void onestep(void);
void onestep1(void);
static void __delay(uint32  uiD);
static void __rightMotorContr(void);
static void __leftMotorContr(void);
static void __mouseCoorUpdate(void);
static void __irSendFreq(int8  __cNumber);
void __irCheck(void);
void ZEROcorrect(void);
static void __wallCheck(void);                                          /*  ǽ�ڼ��                    */
static void __sensorInit(void);
static void __MotorIint(void);
static void __keyInit(void);
static void __sysTickInit(void);
void MouseInit(void);
void __mazeInfDebug (void);
void mouseTurnleft_L(void);                                               /*  ����ת90��                  */
void mouseTurnright_L(void);                                              /*  ����ת90��                  */
void mouseGoahead_L(int8  cNBlock);                                       /*  ǰ��N��                     */
void mouseTurnlefthui(void);                                               /*  ����ת90��                  */
void mouseTurnrighthui(void);
void objectGoTo_CHui(int8 cXdst, int8 cYdst);  
void objectGoTo1(int8  cXdst, int8  cYdst);
void mouseStop(void); 
void mouseGo(void);
void __keyIntDisable (void);
void mouseTurnback_C(void);                                               /*  ���ת                      */
void mouseTurnback_Yqidian(void);
void mouseTurnback_C1(void);                                               /*  ���ת                      */
void mouseTurnleft_C(void);                                               /*  ����ת90��                  */
void mouseTurnright_C(void);                                              /*  ����ת90��                  */
void mouseTurnleft_KC(void);                                               /*  ����ת90��                  */
void mouseTurnright_KC(void);                                              /*  ����ת90��                  */
uint8 DenggaoCheck (void);
uint8 PulseCtrCheck (void);
void mouseTurnleft_Y(void);                                               /*  ����ת90��                  */
void mouseTurnright_Y(void);                                              /*  ����ת90��                  */
uint8 startCheck (void);
void GYRO_Z_Angle(void);
extern void mouseInit (void);
void AD_Init(void);
void test(void);
void mouseGoaheadhui_45 (uint8_t  cNBlock);
void mouseGoaheadhui_kk (uint8_t  cNBlock,uint16 OneStep);
void mouseTurnleft_45(void);
void mouseTurnright_45(void);
void testEncoder(void);


void mouseTurnleft_135_t(void);
void mouseTurnright_135_t(void);
void mouseTurnleft_135_v(void);
void mouseTurnright_135_v(void);
void mouseGoaheadhui_45(uint8_t  );
void mouseGoahead_liang(int8);
void mouseTurnleft_90_t(void);
void mouseTurnright_90_t(void);
void mouseTurnleft_90_v(void);
void mouseTurnright_90_v(void);
void mouseTurnleft_180(void);
void mouseTurnright_180(void);
void mouseTurnleft_45_t(void);
void mouseTurnright_45_t(void);
void mouseTurnleft_45_v(void);
void mouseTurnright_45_v(void);
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
