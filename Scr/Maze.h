/****************************************Copyright (c)****************************************************
**                                �������ΰҵ�Ƽ����޹�˾
**                                     
**                                 http://www.qcmcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           Maze.h
** Last modified Date: 
** Last Version: 
** Description:         �����󶥲���Ƴ���ͷ�ļ�
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


#ifndef __Maze_h
#define __Maze_h


/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "Zlg7289.h"
#include "Type.h"
#include "Micromouse.h"
#include "Mouse_Config.h"
/*********************************************************************************************************
  �����궨�� -- ��������������״̬
*********************************************************************************************************/
#define  WAIT           0                                               /*  �ȴ�״̬                    */
#define  START          1                                               /*  ����״̬                    */
#define  MAZESEARCH     2                                               /*  ��Ѱ״̬                    */
#define  SPURT          3                                               /*  ���״̬                    */
#define  RESTART        4                                               /*  ��������״̬                    */
#define  SPURTL         5                                               /*  ���״̬                    */
#define  RESTARTL       6                                               /*  ��������״̬                    */
#define  SPURTLL         7                                               /*  ���״̬                    */
#define  SPURT45         8 
/*********************************************************************************************************
  ������Ҫʹ�õ��ⲿ����
*********************************************************************************************************/
extern void  mouseInit(void);                                           /*  �ײ����������ʼ��          */
extern void  mouseGoahead(int8  cNBlock);                               /*  ǰ��N��                     */
extern void  mazeSearch(void);                                          /*  �Թ�����                    */
extern void  mouseTurnleft(void);                                       /*  ����ת90��                  */
extern void  mouseTurnright(void);                                      /*  ����ת90��                  */
extern void  mouseTurnback(void);                                       /*  ���ת                      */
extern uint8 keyCheck(void);                                            /*  ��ⰴ��                    */
extern void  sensorDebug(void);                                         /*  ����������                  */
extern void  mouseStop(void);
extern void  __keyIntDisable (void);
extern void  mouseGoahead_Llow (int8  cNBlock);
extern void __UART0Init (void);
extern void mouseTurnleft_C(void);                                               /*  ����ת90��                  */
extern void mouseTurnright_C(void);                                              /*  ����ת90��                  */
extern void mouseTurnback_Y(void);                                               /*  ���ת                      */
extern uint8 startCheck (void);
extern uint32 read_fm24LC16(uint32 *array,uint32 address,uint32 control,uint32 len);
extern void PIDInit(void);
void floodMethodnew(int8 cXdst, int8 cYdst);
void cornerMethodgo(void);
void centralMethodnew(void);
void Sendmap();
void Diagonalway(unsigned char cXdst,unsigned char cYdst);
void objectGoTo_45(uint8 ucStep,uint8  cXdst, uint8  cYdst);
void mouseSpurt_45(void);
void uart_sendchar(unsigned char ch);
/*********************************************************************************************************
  ������Ҫʹ�õ��ⲿ����
*********************************************************************************************************/
extern MAZECOOR GmcMouse;                                               /*  GmcMouse.x :�����������    */
                                                                        /*  GmcMouse.y :������������    */
                                                                        
extern uint8    GucMouseDir;                                            /*  �������ǰ������            */
extern uint8    GucMapBlock[MAZETYPE][MAZETYPE];                        /*  GucMapBlock[x][y]           */
                                                                         /*  x,������;y,������;          */
                                                       /*  bit3~bit0�ֱ������������   */
extern uint8    map;                                                                        /*  0:�÷�����·��1:�÷�����·  */
extern uint8    GucYiBaiBa;                   
extern uint8    GucDirTemp;
extern uint8    GucMapBlock1[MAZETYPE][MAZETYPE];  
extern uint8    GucMapBlock0[MAZETYPE][MAZETYPE];  /*���ˮʱǽ������*/
extern uint8    GucMouseStart;
extern uint8    GucFrontJinju;
extern uint8    GucCrossroad;

extern uint16   GusFreq_F;  
extern uint16   GusFreq_FJ;      
extern uint16   GusFreq_X;      
extern uint16   GusFreq_LF;           
extern uint16   GusFreq_L;                
//������趨ֵ�����ĳ�������ʽ
extern uint16 GusDistance_L_Near; //������
extern uint16 GusDistance_L_Mid;  //�������
extern uint16 GusDistance_L_Far;  //�����Զ
extern uint16 GusDistance_R_Near; //�Һ����
extern uint16 GusDistance_R_Mid;  //�Һ�����
extern uint16 GusDistance_R_Far;  //�Һ����
extern uint16 GusDistance_FL_Near;  //���࣬�����ж�ֹͣ 
extern uint16 GusDistance_FR_Near;
extern uint16 GusDistance_FL_Far;   //Զ�������ж�ǽ�� 
extern uint16 GusDistance_FR_Far;
static void  mapStepEdit(int8  cX, int8  cY);
static void  mouseSpurt(void);
static void  objectGoTo(int8  cXdst, int8  cYdst);
static void  objectGoTo_hui(int8  cXdst, int8  cYdst);
static void  objectGoTo_liang(int8  cXdst, int8  cYdst);
static void  objectGoTo_L(int8  cXdst, int8  cYdst);
static uint8 mazeBlockDataGet(uint8  ucDirTemp);
static void  rightMethod(void);
static void  leftMethod(void);
static void  frontRightMethod(void);
static void  frontLeftMethod(void);
static void  centralMethod(void);
//void actiongenerate();

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
