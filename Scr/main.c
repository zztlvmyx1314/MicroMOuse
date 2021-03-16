/****************************************Copyright (c)****************************************************
**                               �������ΰҵ�Ƽ����޹�˾
**                                     
**                                 http://www.qcmcu.com
**
**                                 Modified by��Chen likao
**--------------File Info---------------------------------------------------------------------------------
** File Name:           maze.c
** Last modified Date:  
** Last Version:        V1.0
** Description:         ���ݵײ����ȡ�õ��Թ���Ϣ�������������㷨���Ƶ��������һ״̬���������ײ�������
**                      ��ִ�С�
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          
** Created date:       
** Version:             V1.0
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**                                  ���ж���stop
*********************************************************************************************************/


/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "Maze.h"
#include "stm32f10x.h"
#include "zlg7289.h"
#include "BitBand.h"
#include "Mouse_Drive.h"
#include "stdio.h"
#include <mouseclub.h>
/*********************************************************************************************************
  ȫ�ֱ�������
*********************************************************************************************************/

extern __PID  __GmSPID;
extern uint8    GucGoHead;
extern uint8    GucGoHead1; 
  uint8    GucXStart                           = 0;                /*  ��������                  */
  uint8    GucYStart                           = 0;                /*  ���������                  */
static uint8    GucXGoal0                           = XDST0;            /*  �յ�X���꣬������ֵ         */
static uint8    GucXGoal1                           = XDST1;
static uint8    GucYGoal0                           = YDST0;            /*  �յ�Y���꣬������ֵ         */
static uint8    GucYGoal1                           = YDST1;
static uint8    GucMouseTask                        = WAIT;             /*  ״̬������ʼ״̬Ϊ�ȴ�      */
 uint8    GucMapStep[MAZETYPE][MAZETYPE]      = {0xff};           /*  ���������ĵȸ�ֵ          */
static uint8    GucMapStep1[MAZETYPE][MAZETYPE]     = {0xff};
static MAZECOOR GmcStack[MAZETYPE * MAZETYPE]       = {0};              /*  ��mapStepEdit()������ջʹ�� */
static MAZECOOR GmcStack1[MAZETYPE * MAZETYPE]      = {0};
static MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE]    = {0};              /*  Main()���ݴ�δ�߹�֧·����  */
uint8 Rece=0;
unsigned char 	MapDiagonalway[16][16]={0xff};
unsigned char   DiagonalwayCount=0;
extern uint8 GoHead45_flag;
uint8 turn45_flag=0;
unsigned char	DiagonalwayAction[255]={0};
unsigned char actionsequence[500]={0};
static uint8    GucMouseTurn                        =0;
  uint8 ffff=0;
  
 uint8 LT90L= 1;
 uint8 RT90L= 2;
 uint8 LV90L= 3;
 uint8 RV90L= 4;
 uint8 LT45L= 5;
 uint8 RT45L =6;
 uint8 LV45L =7;
 uint8 RV45L =8;
 uint8 LT135L= 9;
 uint8 RT135L= 10;
 uint8 LV135L =11;
 uint8 RV135L= 12;
 uint8 L180L  =13;
 uint8 R180L  =14;
 uint8 STRAIGHT90L= 15;        
 uint8 STRAIGHT45L =16;        
 uint8 hh=0;
 
 uint8 zongdianx;
 uint8 zongdiany;
uint8 led_flag;

static uint8  cDirsishiwu;
static   uint8  mmm=0;
static   uint8 nnn=0;
/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        ��ʱ����
** input parameters:    uiD :��ʱ������ֵԽ����ʱԽ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void delay (uint32 uiD)
{
    for (; uiD; uiD--);
}

/*********************************************************************************************************
** Function name:       mapStepEdit
** Descriptions:        ������Ŀ���Ϊ���ĵȸ�ͼ
** input parameters:    uiX:    Ŀ�ĵغ�����
**                      uiY:    Ŀ�ĵ�������
** output parameters:   GucMapStep[][]:  �������ϵĵȸ�ֵ
** Returned value:      ��
*********************************************************************************************************/
void mapStepEdit (int8  cX, int8  cY)
{
    uint8 n         = 0;                                                /*  GmcStack[]�±�              */
    uint8 ucStep    = 1;                                                /*  �ȸ�ֵ                      */
    uint8 ucStat    = 0;                                                /*  ͳ�ƿ�ǰ���ķ�����          */
    uint8 i,j;
    
    GmcStack[n].cX  = cX;                                               /*  ���Xֵ��ջ                 */
    GmcStack[n].cY  = cY;                                               /*  ���Yֵ��ջ                 */
    n++;
    /*
     *  ��ʼ��������ȸ�ֵ
     */
    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            GucMapStep[i][j] = 0xff;
        }
    }
    /*
     *  �����ȸ�ͼ��ֱ����ջ���������ݴ������
     */
    while (n) {
        GucMapStep[cX][cY] = ucStep++;                                  /*  ����ȸ�ֵ                  */

        /*
         *  �Ե�ǰ��������ǰ���ķ���ͳ��
         */
        ucStat = 0;
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  ǰ����·     �洢ǽ����Ϣ���䣬��Ϊ�洢ǽ����Ϣ               */
            (GucMapStep[cX][cY + 1] > (ucStep))) {                      /*  ǰ���ȸ�ֵ���ڼƻ��趨ֵ ����Ϊ�洢�ȸ�ֵ   */
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep[cX + 1][cY] > (ucStep))) {                      /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&
            (GucMapStep[cX][cY - 1] > (ucStep))) {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&
            (GucMapStep[cX - 1][cY] > (ucStep))) {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        /*
         *  û�п�ǰ���ķ�������ת���������ķ�֧��
         *  ������ѡһ��ǰ������ǰ��
         */
        if (ucStat == 0) {
            n--;
            cX = GmcStack[n].cX;
            cY = GmcStack[n].cY;
            ucStep = GucMapStep[cX][cY];
        } else {
            if (ucStat > 1) {                                           /*  �ж����ǰ�����򣬱�������  */
                GmcStack[n].cX = cX;                                    /*  ������Xֵ��ջ               */
                GmcStack[n].cY = cY;                                    /*  ������Yֵ��ջ               */
                n++;
            }
            /*
             *  ����ѡ��һ����ǰ���ķ���ǰ��
             */
            if ((GucMapBlock[cX][cY] & 0x01) &&                         /*  �Ϸ���·                    */
                (GucMapStep[cX][cY + 1] > (ucStep))) {                  /*  �Ϸ��ȸ�ֵ���ڼƻ��趨ֵ    */
                cY++;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x02) &&                         /*  �ҷ���·                    */
                (GucMapStep[cX + 1][cY] > (ucStep))) {                  /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
                cX++;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x04) &&                         /*  �·���·                    */
                (GucMapStep[cX][cY - 1] > (ucStep))) {                  /*  �·��ȸ�ֵ���ڼƻ��趨ֵ    */
                cY--;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x08) &&                         /*  ����·                    */
                (GucMapStep[cX - 1][cY] > (ucStep))) {                  /*  �󷽵ȸ�ֵ���ڼƻ��趨ֵ    */
                cX--;                                                   /*  �޸�����                    */
                continue;
            }
        }
    }
}

void mapStepEdithong (int8  cX, int8  cY)
{
    uint8 n         = 0;                                                /*  GmcStack[]�±�              */
    uint8 ucStep    = 1;                                                /*  �ȸ�ֵ                      */
    uint8 ucStat    = 0;                                                /*  ͳ�ƿ�ǰ���ķ�����          */
    uint8 x = 1;
    uint8 i,j;
    
    GmcStack1[n].cX  = cX;                                               /*  ���Xֵ��ջ                 */
    GmcStack1[n].cY  = cY;                                               /*  ���Yֵ��ջ                 */
    n++;
    /*
     *  ��ʼ��������ȸ�ֵ
     */
    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            GucMapStep1[i][j] = 0xff;
        }
    }
    //mouseStop();
    /*
     *  �����ȸ�ͼ��ֱ����ջ���������ݴ������
     */
    while (n) 
    {
        if(x)
        {
            if(DenggaoCheck() == true)
            {
                if(GucFrontJinju)
                {
                    mouseStop();
                    x=0;
                }
            }
        }
        GucMapStep1[cX][cY] = ucStep++;                                  /*  ����ȸ�ֵ                  */

        /*
         *  �Ե�ǰ��������ǰ���ķ���ͳ��
         *  ��ǰ���Ķ�����˵��ǰ������Ĳ���ֵ�ȵ�ǰ�����ϵ�ֵ��2����
         */
        ucStat = 0;
        if ((GucMapBlock1[cX][cY] & 0x01) &&                             /*  ǰ����·                    */
            (GucMapStep1[cX][cY + 1] > (ucStep)))                        /*  ǰ���ȸ�ֵ���ڼƻ��趨ֵ    */
        {                      
            ucStat++;                                                    /*  ��ǰ����������1             */
        }
        if ((GucMapBlock1[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep1[cX + 1][cY] > (ucStep)))                        /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
        {                      
            ucStat++;                                                    /*  ��ǰ����������1             */
        }
        if ((GucMapBlock1[cX][cY] & 0x04) &&
            (GucMapStep1[cX][cY - 1] > (ucStep))) 
        {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock1[cX][cY] & 0x08) &&
            (GucMapStep1[cX - 1][cY] > (ucStep))) 
        {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        /*
         *  û�п�ǰ���ķ�������ת���������ķ�֧��
         *  ������ѡһ��ǰ������ǰ��
         */
        if (ucStat == 0)
        {
            n--;
            cX = GmcStack1[n].cX;
            cY = GmcStack1[n].cY;
            ucStep = GucMapStep1[cX][cY];
        } 
        else 
        {
            if (ucStat > 1)                                              /*  �ж����ǰ�����򣬱�������  */
            {                                           
                GmcStack1[n].cX = cX;                                    /*  ������Xֵ��ջ               */
                GmcStack1[n].cY = cY;                                    /*  ������Yֵ��ջ               */
                n++;
            }
            /*
             *  ����ѡ��һ����ǰ���ķ���ǰ��
             */
            if ((GucMapBlock1[cX][cY] & 0x01) &&                         /*  �Ϸ���·                    */
                (GucMapStep1[cX][cY + 1] > (ucStep)))                    /*  �Ϸ��ȸ�ֵ���ڼƻ��趨ֵ    */
            {                  
                cY++;                                                    /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock1[cX][cY] & 0x02) &&                         /*  �ҷ���·                    */
                (GucMapStep1[cX + 1][cY] > (ucStep)))                    /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
            {                   
                cX++;                                                    /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock1[cX][cY] & 0x04) &&                         /*  �·���·                    */
                (GucMapStep1[cX][cY - 1] > (ucStep)))                    /*  �·��ȸ�ֵ���ڼƻ��趨ֵ    */
            {                  
                cY--;                                                    /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock1[cX][cY] & 0x08) &&                         /*  ����·                    */
                (GucMapStep1[cX - 1][cY] > (ucStep)))                    /*  �󷽵ȸ�ֵ���ڼƻ��趨ֵ    */
            {                  
                cX--;                                                    /*  �޸�����                    */
                continue;
            }
        }
    }
    GucMouseTurn =1;
}

void mouseSpurt (void)
{
    uint8 ucTemp = 0xff;
    int8 cXdst = 0,cYdst = 0;
    /*
     *  ���յ���ĸ�����ֱ������ȸ�ͼ
     *  ȡ����������һ������ΪĿ���
     */
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    
    objectGoTo1(cXdst,cYdst);                                            /*  ���е�ָ��Ŀ���            */
    
}

void objectGoTo1(int8  cXdst, int8  cYdst)
{
    uint8 ucStep = 1;
    int8  cNBlock = 0, cDirTemp;
    int8 cX,cY;
    GucCrossroad=0;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdit(cXdst,cYdst);                                           /*  �����ȸ�ͼ                  */
    
    /*
     *  ���ݵȸ�ֵ��Ŀ����˶���ֱ���ﵽĿ�ĵ�
     */
    while ((cX != cXdst) || (cY != cYdst)) {
        
        ucStep = GucMapStep[cX][cY];
        /*
         *  ��ѡһ���ȸ�ֵ�ȵ�ǰ����ȸ�ֵС�ķ���ǰ��
         */
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  �Ϸ���·                    */
            (GucMapStep[cX][cY + 1] < ucStep)) {                        /*  �Ϸ��ȸ�ֵ��С              */
            cDirTemp = UP;                                              /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY++;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep[cX + 1][cY] < ucStep)) {                        /*  �ҷ��ȸ�ֵ��С              */
            cDirTemp = RIGHT;                                           /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX++;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  �·���·                    */
            (GucMapStep[cX][cY - 1] < ucStep)) {                        /*  �·��ȸ�ֵ��С              */
            cDirTemp = DOWN;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY--;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  ����·                    */
            (GucMapStep[cX - 1][cY] < ucStep)) {                        /*  �󷽵ȸ�ֵ��С              */
            cDirTemp = LEFT;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX--;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        
        cDirTemp = (cDirTemp + 8 - GucMouseDir)%8;                      /*  ���㷽��ƫ����              */
        GucDirTemp = cDirTemp;
        if (cNBlock) {
          if((GucCrossroad <= 1)&&(cNBlock>1))
              mouseGoahead_L(cNBlock);                                      /*  ǰ��cNBlock��               */
          else{
            mouseGoahead_L(cNBlock);
            GucCrossroad = 0;
          }
          
          
        }        
        cNBlock = 0;  
        /*  ��������                    */
        
        /*
         *  ���Ƶ�����ת��
         */
         
       switch (cDirTemp) {

        case 2:
            mouseTurnright_C();
            break;

        case 4:
            mouseTurnback();
            break;

        case 6:
            mouseTurnleft_C();
            break;

        default:
            break;
        }
      GmcMouse.cX=cX;
      GmcMouse.cY=cY;
    }
    /*
     *  �ж������Ƿ���ɣ��������ǰ��
     */
    
      if (cNBlock) {
          if((GucCrossroad <= 1)&&(cNBlock>1))
            mouseGoahead_L(cNBlock);      //                                /*  ǰ��cNBlock��               */
          else{
            mouseGoahead_L(cNBlock);
            GucCrossroad = 0;
          }
          GmcMouse.cX=cX;
          GmcMouse.cY=cY;
      }
}


char chongci=0;
void mouseSpurt_CC (void)
{
    uint8 ucTemp = 0xff;
    int8 cXdst = 0,cYdst = 0;
    //  ���յ���ĸ�����ֱ������ȸ�ͼ
    //  ȡ����������һ������ΪĿ���
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    
    chongci=1;
    objectGoTo1(cXdst,cYdst);
                                           /*  ���е�ָ��Ŀ���            */
    
}



void mouseSpurt_CCC (void)
{
    uint8 ucTemp = 0xff;
    int8 cXdst = 0,cYdst = 0;
    //  ���յ���ĸ�����ֱ������ȸ�ͼ
    //  ȡ����������һ������ΪĿ���
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    
    chongci=1;
    objectGoTo1(cXdst,cYdst);                                            /*  ���е�ָ��Ŀ���            */
    
}




void mouseSpurt_45(void)
{
    uint8 ucTemp = 0xff;
    uint8 cXdst = 0,cYdst = 0;
    /*
     *  ���յ���ĸ�����ֱ������ȸ�ͼ
     *  ȡ����������һ������ΪĿ���  
     */
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    mapStepEdit(cXdst,cYdst);                                           /*  �����ȸ�ͼ                  */
    Diagonalway(cXdst,cYdst);    
    actiongenerate();
    GucMouseDir=0;
    objectGoTo_45(ucTemp,cXdst,cYdst);
    GmcMouse.cX=cXdst;
    GmcMouse.cY=cYdst;
 
}


void mouseSpurt_C (void)
{
    uint8 ucTemp = 0xff;
    int8 cXdst = 0,cYdst = 0;
    //  ���յ���ĸ�����ֱ������ȸ�ͼ
    //  ȡ����������һ������ΪĿ���
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03)                       /*  �жϸ��յ������Ƿ��г���    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  ������������������        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    
    objectGoTo_CHui(cXdst,cYdst);     //objectGoTo_Hui(cXdst,cYdst);                  /*  ���е�ָ��Ŀ���            */
    
}




int getsum(int num)
{
	int ret=0,i=0;
	for(i=0;i<=num;i++) ret+=DiagonalwayAction[i];
	return ret;
}


uint8 xiexianyige=0;
int action=0;
void objectGoTo_45(uint8 ucStep,uint8  cXdst, uint8  cYdst)
{  
   int i=0,num=0;
    int a=0;   
 

    /*
     *  ���ݵȸ�ֵ��Ŀ����˶���ֱ���ﵽĿ�ĵ�
     */


    		while(actionsequence[action]!=0)
		{
			if(actionsequence[action]==16)
			{	
				num=0;
				while(actionsequence[action]==16)
				{
				 	num++;
					action++;
				}
                                 mouseGoaheadhui_45(num);                            
			}
			else if(actionsequence[action]==15)
			{	
				num=0;
				while(actionsequence[action]==15)
				{
				 	num++;
					action++;
				} 
                                mouseGoahead_liang(num);                                 
			}
                       
			else
			{	
                                         switch(actionsequence[action])
                                          {
                     
                                               case 1://LT90L������ת          
                                                          
                                                         //mouseTurnleft_90_t();  
                                                       mouseTurnleft_90_t();  
                                                       
                                                         ffff=ffff+6;
                                                         //mouseGoaheadhui_kk(1,42000);    
                                                         break;

                                               case 2:// RT90L������ת            
                                                        ffff=ffff+2;
                                                      
                                                          mouseTurnright_90_t();
                                                           
                                                         //  mouseGoaheadhui_kk(1,42000);   
                                                      
                                                          break;
                                                case 3 ://LV90L   б����ת90��'       
                                                  ffff=ffff+6;
                                                           GoHead45_flag=2;
                                                           mouseTurnleft_90_v();        
                                                          GoHead45_flag=0;
                                                         break;
                                                case 4://RV90L    б����ת90��        
                                                 ffff=ffff+2;
                                                         GoHead45_flag=2;
                                                         
                                                         mouseTurnright_90_v(); 
                                                          //mouseTurnright_90_v();        
                                                          GoHead45_flag=0;
                                                         
                                                          break;
                                                 case 5: //LT45L      ֱ����ת45              ok
                                                   ffff=ffff+7;
                                                          GoHead45_flag=2;
                                                          mouseTurnleft_45_t();     
                                                         // mouseTurnleft_45_t();        
                                                          GoHead45_flag=0;
                                                           break;
                                                 case 6://RT45L       ֱ����ת45      ok
                                                   ffff=ffff+1;
                                                   
                                                          GoHead45_flag=2;//��У��
                                                          mouseTurnright_45_t();     
                                                          GoHead45_flag=0;///б��У��
                                                         
                                                            break;  
                                                 case 7://LV45L        б����ת45��     
                                                   ffff=ffff+7;
                                                           //mouseTurnleft_45_v();
                                                  // mouseStop();while(1);
                                                  mouseTurnleft_45_v();
                                                          
                                                                                                       ;
                                                          break;  
                                                 case  8://RV45L         б����ת45��   ok
                                                          ffff=ffff+1; 
                                                         mouseTurnright_45_v(); 
                                                          
                                                         break;

                                               case  9:  //LT135L       ֱ����ת135          
                                                 ffff=ffff+5;
                                                          GoHead45_flag=2;
                                                          //mouseTurnleft_135_t(); 
                                                         mouseTurnleft_135_t(); 
                                                          
                                                          
                                                          GoHead45_flag=0;
                                                          break;
                                                case 10://RT135L        ֱ����ת135      
                                                  ffff=ffff+3;
                                                          GoHead45_flag=2;
                                                          mouseTurnright_135_t();          
                                                          GoHead45_flag=0;
                                                         break;
                                                case 11://LV135L        б����ת135      
                                                ffff=ffff+5;
                                                          
                                                          mouseTurnleft_135_v();
                                                         
                                                          break;
                                                 case 12: //RV135L      б����ת135                                                 
                                                         
                                                         ffff=ffff+3;
                                                          //mouseTurnright_135_v();  
                                                        mouseTurnright_135_v();       
                                                         
                                                           break;
                                                           
                                                 case 13:       //��ת180          
                                                         ffff=ffff+4;
                                                           mouseTurnleft_180();      
                                                         //mouseTurnleft_180();        
                                                            break;  
                                                 case  14:      //��ת180            
                                                        ffff=ffff+4;
                                                        mouseTurnright_180();      
                                                        //  mouseTurnright_180();        
                                                                                                                   
                                                            break;   
                                             
                                                default:
                                                  mouseStop();
                                                  while(1);
                                                             break;
                                              }                                                     
                                        action++;                               
			}
		}
  
}

void mouseSpurtbucang() 
{
   GmcMouse.cX=zongdianx;
  GmcMouse.cY=zongdiany;
 //  GmcMouse.cX=7;
 //  GmcMouse.cY=8;
   GucXStart= 0;              
   GucYStart= 0; 
}
int8 lingshix,lingshiy=0;
uint8 lingshixyflag=1;

extern uint8 final;
void objectGoTo_Hui(int8 cXdst, int8 cYdst)//if��˳������޸ģ���һ��if�ŷ�����ʼ��������ķ���
{
    uint8 ucStep = 1;
    int8  cNBlock = 0, cDirTemp;
    int8  cX,cY;
    GucCrossroad = 0;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdit(cXdst, cYdst);                                           /*  �����ȸ�ͼ                  */
    
    while ((cX != cXdst) || (cY != cYdst)) 
    {
        ucStep = GucMapStep[cX][cY];
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  �Ϸ���·                    */
            (GucMapStep[cX][cY + 1] < ucStep))                          /*  �Ϸ��ȸ�ֵ��С              */
        {                        
            cDirTemp = UP;                                              /*  ��¼����                    */
            if (cDirTemp == GucMouseDir)                                /*  ����ѡ����Ҫת��ķ���    */
            {                              
                cNBlock++;                                              /*  ǰ��һ������                */
                cY++;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep[cX + 1][cY] < ucStep))                          /*  �ҷ��ȸ�ֵ��С              */
        {                        
            cDirTemp = RIGHT;                                           /*  ��¼����                    */
            if (cDirTemp == GucMouseDir)                                /*  ����ѡ����Ҫת��ķ���    */
            {                             
                cNBlock++;                                              /*  ǰ��һ������                */
                cX++;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  �·���·                    */
            (GucMapStep[cX][cY - 1] < ucStep))                          /*  �·��ȸ�ֵ��С              */
        {                        
            cDirTemp = DOWN;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir)                                /*  ����ѡ����Ҫת��ķ���    */
            {                              
                cNBlock++;                                              /*  ǰ��һ������                */
                cY--;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  ����·                    */
            (GucMapStep[cX - 1][cY] < ucStep))                          /*  �󷽵ȸ�ֵ��С              */
        {                        
            cDirTemp = LEFT;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir)                                /*  ����ѡ����Ҫת��ķ���    */
            {                              
                cNBlock++;                                              /*  ǰ��һ������                */
                cX--;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        cDirTemp = (cDirTemp + 8 - GucMouseDir)%8;                      /*  ���㷽��ƫ����              */
        GucDirTemp = cDirTemp;
        if (cNBlock) 
        {
          
                mouseGoahead(cNBlock);                                      /*  ǰ��cNBlock��               */
         
        }        
        cNBlock = 0;  
        /*  ��������                    */
        
       switch (cDirTemp) 
       {
           case 1:    
               mouseTurnright_Hui();
               break;

           case 2:
               mouseTurnback();
               break;

           case 3:
               mouseTurnleft_Hui();
               break;

           default:
               break;
        }
        GmcMouse.cX=cX;
        GmcMouse.cY=cY;
    }
    
    if (cNBlock) 
    {
            mouseGoahead(cNBlock);                                  //    ǰ��cNBlock��      
       
        GmcMouse.cX=cX;
        GmcMouse.cY=cY;
    }
}

/*********************************************************************************************************
** Function name:       mazeBlockDataGet
** Descriptions:        ���ݵ��������Է���ȡ���÷������Թ����ǽ������
** input parameters:    ucDir: ���������Է���
** output parameters:   ��
** Returned value:      GucMapBlock[cX][cY] : ǽ������
*********************************************************************************************************/
uint8 mazeBlockDataGet (uint8  ucDirTemp)
{
    int8 cX = 0,cY = 0;
    
    /*
     *  �ѵ��������Է���ת��Ϊ���Է���
     */
    switch (ucDirTemp) {

    case MOUSEFRONT:
        ucDirTemp = GucMouseDir;
        break;

    case MOUSELEFT:
        ucDirTemp = (GucMouseDir + 6) % 8;
        break;

    case MOUSERIGHT:
        ucDirTemp = (GucMouseDir + 2) % 8;
        break;

    default:
        break;
    }
    
    /*
     *  ���ݾ��Է������÷��������ڸ������
     */
    switch (ucDirTemp) {

    case 0:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY + 1;
        break;
        
    case 2:
        cX = GmcMouse.cX + 1;
        cY = GmcMouse.cY;
        break;
        
    case 4:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY - 1;
        break;
        
    case 6:
        cX = GmcMouse.cX - 1;
        cY = GmcMouse.cY;
        break;
        
    default:
        break;
    }
    
    return(GucMapBlock0[cX][cY]);                                        /*  �����Թ����ϵ�����          */
}
/*********************************************************************************************************
** Function name:       rightMethod
** Descriptions:        ���ַ�����������ǰ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void rightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        //mouseStop();while(1);
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       leftMethod
** Descriptions:        ���ַ������������˶�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void leftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontRightMethod
** Descriptions:        ���ҷ���������ǰ���У��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void frontRightMethod (void)
 {
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontLeftMethod
** Descriptions:        ������������ǰ���У��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void frontLeftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
}

/*********************************************************************************************************
** Function name:       centralMethod
** Descriptions:        ���ķ��򣬸��ݵ�����Ŀǰ���Թ���������λ�þ���ʹ�ú�����������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void centralMethod (void)
{
    if (GmcMouse.cX & 0x08) {
        if (GmcMouse.cY & 0x08) {

            /*
             *  ��ʱ���������Թ������Ͻ�
             */ 
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  ��ʱ���������Թ������½�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            default:
                break;
            }
        }
    } else {
        if (GmcMouse.cY & 0x08) {

            /*
             *  ��ʱ���������Թ������Ͻ�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  ��ʱ���������Թ������½�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            default:
                break;
            }
        }
    }
}
/*********************************************************************************************************
** Function name:       crosswayCheck
** Descriptions:        ͳ��ĳ������ڻ�δ�߹���֧·��
** input parameters:    ucX����Ҫ����ĺ�����
**                      ucY����Ҫ�����������
** output parameters:   ��
** Returned value:      ucCt��δ�߹���֧·��
*********************************************************************************************************/
uint8 crosswayCheck (int8  cX, int8  cY)
{
    uint8 ucCt = 0;
    if ((GucMapBlock[cX][cY] & 0x01) &&                                 /*  ���Է����Թ��Ϸ���·      */
        (GucMapBlock0[cX][cY + 1]) == 0x00) {                            /*  ���Է����Թ��Ϸ�δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x02) &&                                 /*  ���Է����Թ��ҷ���·      */
        (GucMapBlock0[cX + 1][cY]) == 0x00) {                            /*  ���Է����Թ��ҷ�û���߹�  */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x04) &&                                 /*  ���Է����Թ��·���·      */
        (GucMapBlock0[cX][cY - 1]) == 0x00) {                            /*  ���Է����Թ��·�δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x08) &&                                 /*  ���Է����Թ�����·      */
        (GucMapBlock0[cX - 1][cY]) == 0x00) {                            /*  ���Է����Թ���δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    return ucCt;
}
/*********************************************************************************************************
** Function name:       crosswayChoice
** Descriptions:        ѡ��һ��֧·��Ϊǰ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void crosswayChoice (void)
{
    switch (2) {
        
    case RIGHTMETHOD:
        rightMethod();
        break;
    
    case LEFTMETHOD:
        leftMethod();
        break;
    
    case CENTRALMETHOD:
        centralMethod();
        break;

    case FRONTRIGHTMETHOD:
        frontRightMethod();
        break;

    case FRONTLEFTMETHOD:
        frontLeftMethod();
        break;
       

    default:
        break;
    }
}
/*********************************************************************************************************
** Function name:       __ir_Get
** Descriptions:        ��ȡE2PROM�еĺ���Ƶ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void RCC_Init(void)
{    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | 
                           RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE);
}

// mode:JTAG,SWDģʽ���ã�00��ȫʹ�ܣ�01ʹ��SWD��10ȫ�ر�
void JTAG_Set(u8 mode)
{
    u32 temp;
    temp=mode;
    temp<<=25;
    RCC->APB2ENR|=1<<0;
    AFIO->MAPR&=0XF8FFFFFF;
    AFIO->MAPR|=temp;
}

void centralMethodnew(void)
{
    int m = 0,flag = 0;                                                /*  ͳ�ƿ�ǰ����֧·��          */
    static int ss=0;
    
    while(1)
    {
        if((((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal0))|| 
           ((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal1)) ||
           ((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal1)) ||
           ((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal0))))
        {
             goto End2;
        }
        m = crosswayCheck(GmcMouse.cX,GmcMouse.cY);
        if (m>0)                                                       /*  �п�ǰ������                */
        {  if(m>1){ ss++; if(ss=0){mouseStop();while(1);}};
            
        crosswayChoice();                                          /*  �����ַ�������ѡ��ǰ������  */
        
            mazeSearch();
        }
        else if (m==0)
        {
            flag=1;      
            goto End2;
        }
        else;
      }
End2: if(flag)
          floodMethodnew(GucXGoal0, GucYGoal0);
      else;
}

void floodMethodnew(int8 cXdst, int8 cYdst)
{


    uint8 flag = 0;
    uint8 n = 0;
    uint8 ucStep = 0;
    int8  cDirTemp;
    int8  cX,cY;
    int8  t=0;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdithong(cXdst,cYdst);
    if(((cXdst!=GucXGoal0)&&(cXdst!=GucXGoal1))||
       ((cYdst!=GucYGoal0)&&(cYdst!=GucYGoal1)))             
    {           
        t=1;                                                         
    }
    while ((cX!=cXdst)||(cY!=cYdst)) 
    {
        if((((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal0)) || 
            ((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal1)) ||
            ((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal1)) ||
            ((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal0)))
            &&(t==0))
        { 
            cDirTemp=GucMouseDir;
            break;
        }
    
        n=0;
        ucStep = GucMapStep1[cX][cY]; 
        /*
         *  ��ѡһ���ȸ�ֵ�ȵ�ǰ����ȸ�ֵС�ķ���ǰ��
         */
        if ((GucMapBlock1[cX][cY] & 0x01) &&                             /*  �Ϸ���·                    */
            (GucMapStep1[cX][cY + 1] < ucStep)&&                         /*  �Ϸ��ȸ�ֵ��С              */
            (GucMapBlock0[cX][cY + 1]> 0x00))
        {   
            cDirTemp = UP;
            n++;
            if (cDirTemp == GucMouseDir)                                 /*  ����ѡ����Ҫת��ķ���    */
            {                             
                goto T;                                     
            }
        }
        if ((GucMapBlock1[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep1[cX + 1][cY] < ucStep)&&                         /*  �ҷ��ȸ�ֵ��С              */
            (GucMapBlock0[cX + 1][cY] >0x00)) 
        {   
            cDirTemp = RIGHT;
            n++;
            if (cDirTemp == GucMouseDir)                                 /*  ����ѡ����Ҫת��ķ���    */ 
            {                                         
                goto T; 
            }
        }
        if ((GucMapBlock1[cX][cY] & 0x04) &&                             /*  �·���·                    */
            (GucMapStep1[cX][cY - 1] < ucStep)&&                         /*  �·��ȸ�ֵ��С              */
            (GucMapBlock0[cX][cY - 1]>0x00)) 
        {   
            cDirTemp = DOWN;
            n++;
            if (cDirTemp == GucMouseDir)                                 /*  ����ѡ����Ҫת��ķ���    */ 
            {                                           
                goto T; 
            }
        }
        if ((GucMapBlock1[cX][cY] & 0x08) &&                             /*  ����·                    */
            (GucMapStep1[cX - 1][cY] < ucStep)&&                         /*  �󷽵ȸ�ֵ��С              */
            (GucMapBlock0[cX - 1][cY]>0x00)) 
        {
            cDirTemp = LEFT;
            n++;
            if (cDirTemp == GucMouseDir)                                 /*  ����ѡ����Ҫת��ķ���    */
            {                              
                goto T;                
            }
        }      
        if(n==0)
        {
            flag=1;
            goto End1;
        }
        cDirTemp = (cDirTemp + 8 - GucMouseDir)%8;                       /*  ���㷽��ƫ����              */
           
        switch (cDirTemp) 
        {

        case 2:
          if(GucMouseTurn){
            if(DenggaoCheck ()== false){
               while(PulseCtrCheck ()== false);
               mouseStop();
               mouseTurnright_Y();
            }
            GucMouseTurn =0;
          }
          else
            mouseTurnright();
           cX = GmcMouse.cX;
           cY = GmcMouse.cY;
             break;
         case 4:
            GucMouseTurn =0;
            mouseTurnback_H();
            cX = GmcMouse.cX;
            cY = GmcMouse.cY;
         
            break;

        case 6:
          if(GucMouseTurn){
            if(DenggaoCheck ()== false){
               while(PulseCtrCheck ()== false);
               mouseStop();
               mouseTurnleft_Y();
            }
            GucMouseTurn =0;
          }
          else
            
            mouseTurnleft();
            cX = GmcMouse.cX;
            cY = GmcMouse.cY;
            break;

        default:
            break;
        }
    
T:   mazeSearch();
     cX = GmcMouse.cX;
     cY = GmcMouse.cY;
      
    }

End1: if(flag)
        centralMethodnew();
      else;
}

/*********************************************************************************************************
** Function name:       goalwall
** Descriptions:       
** input parameters:    ��
** output parameters:   ��
** Returned value:     
*********************************************************************************************************/
void goalwall(void)
{
 
    if ((GmcMouse.cX==7)&&(GmcMouse.cY==7))                   /* �յ�ǽ�����ϸ�ֵ*/
    {                               
        if(GucMouseDir==UP) 
        {       
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][7]=0x19;
            GucMapBlock0[8][8]=0x1c;
       
        }
        if(GucMouseDir==RIGHT) 
        {
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][7]=0x19;
            GucMapBlock0[8][8]=0x1c;
        }
    }
    if ((GmcMouse.cX==7)&&(GmcMouse.cY==8)) 
    {
        if(GucMouseDir==DOWN) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[8][7]=0x19;
            GucMapBlock0[8][8]=0x1c;
        }
        if(GucMouseDir==RIGHT) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[8][7]=0x19;
            GucMapBlock0[8][8]=0x1c;
        }
    }
    if ((GmcMouse.cX==8)&&(GmcMouse.cY==7)) 
    {
        if(GucMouseDir==UP) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][8]=0x1c;
        }
        if(GucMouseDir==LEFT) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][8]=0x1c;
        }
    }
    if ((GmcMouse.cX==8)&&(GmcMouse.cY==8)) 
    {
        if(GucMouseDir==DOWN) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][7]=0x19;
        }
        if(GucMouseDir==LEFT) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][7]=0x19;
        }
    }
}

void floodwall(void)
{     uint8 n          = 0;                                                                                            
      uint8 ucTemp     = 0;
        for(n=0;n<MAZETYPE;n++) {                                           /*ǽ�ڳ�ֵ�޷�д�룬�����ڴ˴��¸�ֵ*/
          for(ucTemp=0;ucTemp<MAZETYPE;ucTemp++) {                                         /* ��ˮ�㷨����*/
          GucMapBlock1[n][ucTemp] =0x1f; }}
           n=0;    ucTemp     = 0;
       for(n=0;n<MAZETYPE;n++) {
         GucMapBlock1[0][n] =0x17;          
         GucMapBlock1[MAZETYPE-1][n] =0x1d; }
            n=0;
       for(n=0;n<MAZETYPE;n++) 
       {   GucMapBlock1[n][0] =0x1b;          
         GucMapBlock1[n][MAZETYPE-1] =0x1e; }
               n=0;
       GucMapBlock1[MAZETYPE-1][MAZETYPE-1] =0x1c;
        GucMapBlock1[0][MAZETYPE-1] =0x16;
         GucMapBlock1[0][0] =0x11;
         GucMapBlock1[MAZETYPE-1][0] =0x19;
    
             for(n=0;n<MAZETYPE;n++)                                            /*ǽ�ڳ�ֵ�޷�д�룬�����ڴ˴��¸�ֵ*/
       { for(ucTemp=0;ucTemp<MAZETYPE;ucTemp++)
     {  GucMapBlock[n][ucTemp] =0x10; }}
           n=0;    ucTemp     = 0;
}

void StartSave()
{
    Test_Write(START_SAVE_ADDRESS,GucXStart);
}

void wallsave(void)
{
   uint8 x = 0;                                                                                            
   uint8 y = 0;
   uint16 GucMapBlock4[16][16]={0};
   for(x=0;x<MAZETYPE;x++) {                                           
      for(y=0;y<MAZETYPE;y++) {                                         
          GucMapBlock4[x][y] =GucMapBlock[x][y]; 
      }
   } 
  STMFLASH_Write(MAPE_SAVE_ADDRESS,(u16*)GucMapBlock4,256);

}


void wallget(void)
{
   uint8 x = 0;                                                                                            
   uint8 y = 0;
   uint16 GucMapBlock4[16][16]={0};
   STMFLASH_Read(MAPE_SAVE_ADDRESS,(u16*)GucMapBlock4,256);
   delay(10000);
   for(x=0;x<MAZETYPE;x++) {                                           
      for(y=0;y<MAZETYPE;y++) {                                         
          GucMapBlock[x][y] =(uint8)GucMapBlock4[x][y]; 
      }
   }   
}

void StartGet(void)
{
  uint16 start = 0;
 
 // read_fm24LC16(&start,0x80,0xa1,1);
  STMFLASH_Read(START_SAVE_ADDRESS,(u16*)&start,1);
  if(start == 15)
  {
    GucXStart   = MAZETYPE - 1;                             /*  �޸ĵ��������ĺ�����      */
    GmcMouse.cX = MAZETYPE - 1; 
  }
}


void __ir_Get(void)
{    
      uint16    DIS1[10] = {0};
      STMFLASH_Read(IR_SAVE_ADDRESS,(u16 *)DIS1,10);
      delay(1000000);
      GusDistance_L_Near=DIS1[0];
      GusDistance_R_Near=DIS1[1];
      GusDistance_L_Mid=DIS1[2];
      GusDistance_R_Mid=DIS1[3]; 
      GusDistance_L_Far=DIS1[4];
      GusDistance_R_Far=DIS1[5];
      GusDistance_FL_Far=DIS1[6];
      GusDistance_FR_Far=DIS1[7];
      GusDistance_FL_Near=DIS1[8];
      GusDistance_FR_Near=DIS1[9];
}
void GPIO_Config1()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);		   

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	        
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}	




void USART1_Config1(void)
{
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure); 
    USART_Cmd(USART3, ENABLE);
}

void NVIC_Config1(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 		    //????2????��??��??�ꡧ?��??��??��??��?��?2??������??��??�ꡧ?����|��??��??��?
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  	            //USART1?D??�����̨�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 	    //io��y???D???��??��??���̨�???a�̨�?t???��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 		    //io��y???D??������??���̨�???a�̨�?t???��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);		
    USART_Cmd(USART3, ENABLE);
}

void uart_sendchar(unsigned char ch)
{
  while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);
  USART_SendData(USART3,ch);
}


int fputc(int ch,FILE *f)
{
  USART_SendData(USART3,(unsigned char)ch);
  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
  return (ch);
}


void USART3_IRQHandler(void)
{
    int RX_status;
    RX_status = USART_GetFlagStatus(USART3, USART_FLAG_RXNE);
    if(RX_status == SET) 
    {
        //USART_SendData(USART3 , USART_ReceiveData(USART3));
        Rece=USART_ReceiveData(USART3);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);
    }
}
void SendBlook()
{
    uint8 String[]="0123456789abcdef";
    //uint8 shi,ge;
    SendStr("GucMapBlock[");
    uart_sendchar(String[GmcMouse.cY]);
    SendStr("]= ");
    uart_sendchar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X08) >> 3]);
    uart_sendchar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X04) >> 2]);
    uart_sendchar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X02) >> 1]);
    uart_sendchar(String[GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X01]);
    SendStr("         ");
}
void Sendmap()
{
    uint8 String[]="0123456789abcdef";
  /*  if(map == 0)
       uart_sendchar(String[15]);
    else*/
    uart_sendchar(String[GmcMouse.cX]);
    SendStr("  ");
    uart_sendchar(String[GmcMouse.cY]);
    SendStr("  ");
    uart_sendchar(String[GucMapBlock[GmcMouse.cX][GmcMouse.cY]]);
    SendStr("\r\n");
}
void SendStr(uint8 *s)
{
  while(*s){    
  // USART_SendData(USART3, *s++);
    uart_sendchar(*s++);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
void objectGoTo_liang(int8 cXdst, int8 cYdst)
{
    uint8 ucStep = 1;
    int8  cNBlock = 0, cDirTemp;
    int8  cX,cY;
    GucCrossroad = 0;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdit(cXdst, cYdst);                                           /*  �����ȸ�ͼ                  */
    
    // ���ݵȸ�ֵ��Ŀ����˶���ֱ���ﵽĿ�ĵ�
    while ((cX != cXdst) || (cY != cYdst)) 
    {
        ucStep = GucMapStep[cX][cY];
        // ��ѡһ���ȸ�ֵ�ȵ�ǰ����ȸ�ֵС�ķ���ǰ��
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  �Ϸ���·                    */
            (GucMapStep[cX][cY + 1] < ucStep))                          /*  �Ϸ��ȸ�ֵ��С              */
        {                        
            cDirTemp = UP;                                              /*  ��¼����                    */
            if (cDirTemp == GucMouseDir)                                /*  ����ѡ����Ҫת��ķ���    */
            {                              
                cNBlock++;                                              /*  ǰ��һ������                */
                cY++;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep[cX + 1][cY] < ucStep))                          /*  �ҷ��ȸ�ֵ��С              */
        {                        
            cDirTemp = RIGHT;                                           /*  ��¼����                    */
            if (cDirTemp == GucMouseDir)                                /*  ����ѡ����Ҫת��ķ���    */
            {                             
                cNBlock++;                                              /*  ǰ��һ������                */
                cX++;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  �·���·                    */
            (GucMapStep[cX][cY - 1] < ucStep))                          /*  �·��ȸ�ֵ��С              */
        {                        
            cDirTemp = DOWN;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir)                                /*  ����ѡ����Ҫת��ķ���    */
            {                              
                cNBlock++;                                              /*  ǰ��һ������                */
                cY--;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  ����·                    */
            (GucMapStep[cX - 1][cY] < ucStep))                          /*  �󷽵ȸ�ֵ��С              */
        {                        
            cDirTemp = LEFT;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir)                                /*  ����ѡ����Ҫת��ķ���    */
            {                              
                cNBlock++;                                              /*  ǰ��һ������                */
                cX--;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        cDirTemp = (cDirTemp + 8 - GucMouseDir)%8;                      /*  ���㷽��ƫ����              */
        GucDirTemp = cDirTemp;
        if (cNBlock) 
        {
            if((GucCrossroad <= 1)&&(cNBlock >=1))
                mouseGoahead(cNBlock);                                      /*  ǰ��cNBlock��               */
            else
            {
                mouseGoahead(cNBlock);
                GucCrossroad = 0;
            }                 
        }        
        cNBlock = 0;  
        /*  ��������                    */
        
        //  ���Ƶ�����ת��         
       switch (cDirTemp) 
       {
           case 1:    
               mouseTurnright();
               //while(1);
               break;

           case 2:
               mouseTurnback();
               break;

           case 3:
               mouseTurnleft();
               //while(1);
               break;

           default:
               break;
        }
        GmcMouse.cX=cX;
        GmcMouse.cY=cY;
    }
    //  �ж������Ƿ���ɣ��������ǰ��
    
    if (cNBlock) 
    {
        if((GucCrossroad <= 1) && (cNBlock >= 1))
            mouseGoahead(cNBlock);                                     /*  ǰ��cNBlock��               */
        else
        {
            mouseGoahead(cNBlock);
            GucCrossroad = 0;
        }
        GmcMouse.cX=cX;
        GmcMouse.cY=cY;
    }
}


uint8 zijiaozheng0=0;
uint8 usepiancha =0;

main (void)
{
  
 
  
    uint8 n          = 0;                                               /*  GmcCrossway[]�±�           */
    uint8 ucRoadStat = 0;                                               /*  ͳ��ĳһ�����ǰ����֧·��  */
    uint8 ucTemp     = 0;                                               /*  ����START״̬������ת�� */
    uint8 start=0;
    uint8 start_maxspeed=0;
    uint8 start_led=0;
   // extern uint8 quzabo;
    SystemInit();
    RCC_Init();
    JTAG_Set(1);
    MouseInit();
    PIDInit(); 
    ZLG7289Init();
    delay(100000);
    //__ir_Get();
    delay(100000);
    GPIO_Config1();
    USART1_Config1();
    NVIC_Config1();
    floodwall();
    GPIO_SetBits(GPIOB,GPIO_Pin_12);
 
  
    
    while (1) {
        switch (GucMouseTask) {                                         /*  ״̬������                  */     
            case WAIT:
               sensorDebug();
               delay(10000);   
            
             if (startCheck() == true) 
               {       
                  start++;
                }
               if((start==1)&&GucGoHead)//&&GucGoHead
               {
                  start=0;
                  zijiaozheng0=1;
                  
                   GPIO_ResetBits(GPIOB,GPIO_Pin_12);
               
            delay(50000000); 
            GucMouseTask = START;
                  delay(1000000);
                  GPIO_SetBits(GPIOB,GPIO_Pin_12);
                  zijiaozheng0=1;
               }
               if((start<5)&&(start>=3)&&GucGoHead)
               {
                
                 GPIO_ResetBits(GPIOB,GPIO_Pin_12);
                 zijiaozheng0=1;
                  start=0;
              wallget();
              delay(10000000);
              StartGet(); 
              delay(10000000);
              delay(1000000);
              delay(1000000);delay(1000000); delay(1000000);
              GucMouseTask = SPURTL; 
                  delay(1000000);
               }           
                 break;
        case START:                                                     /*  �жϵ��������ĺ�����      */
              GPIO_ResetBits(GPIOB,GPIO_Pin_12);
  
            mazeSearch();                                  /*  ��ǰ����                    */
                if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) 
                {          
                    if (MAZETYPE == 16)                           
                    {                                    
                        GucXGoal0 = 8;
                        GucXGoal1 = 7;
                    }
                    GucXStart   = MAZETYPE - 1;                       
                    GmcMouse.cX = MAZETYPE - 1;                      
                    ucTemp = GmcMouse.cY;
                    do {
                           GucMapBlock[MAZETYPE - 1][ucTemp] = GucMapBlock[0][ucTemp];
                           GucMapBlock0[MAZETYPE - 1][ucTemp]= GucMapBlock0[0][ucTemp];
                           GucMapBlock0[0][ucTemp] = 0;
                           GucMapBlock1[MAZETYPE - 1][ucTemp]= GucMapBlock1[0][ucTemp];    
                           if(ucTemp > 0)
                           {
                               GucMapBlock1[MAZETYPE - 2][ucTemp-1] = 0x1d;
                           }
                           GucMapBlock1[0][ucTemp+1] =0x17;
                           GucMapBlock1[1][ucTemp] =0x1f;
                           GucMapBlock[0][ucTemp] =0x10;
                           GucMapBlock[1][ucTemp] =0x10;                                                
                       }while (ucTemp--);
                    GucMapBlock1[0][0] =0x13;
                    GucMapBlock1[1][0] =0x1b;
                    GucMapBlock1[MAZETYPE - 2][0] =0x19;
                    // ��OFFSHOOT[0]�б����������
                    GmcCrossway[n].cX = MAZETYPE - 1;
                    GmcCrossway[n].cY = 0;
                    n++;
                    GucMouseTask = MAZESEARCH;                              
                }
                if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02)          
                {        
                    // ��OFFSHOOT[0]�б����������
                    GmcCrossway[n].cX = 0;
                    GmcCrossway[n].cY = 0;
                    n++;
                    GucMouseTask = MAZESEARCH;                              
                }
            break;       
        case MAZESEARCH: 
       
            centralMethodnew();
             goalwall();
             StartSave();
             wallsave();
            // onestep1();

             mouseTurnback();
             objectGoTo1(GucXStart,GucYStart);
             
             onestep3();
             mouseTurnbackqi(); 
       
             GucMouseTask = SPURTL;
           
            break;      
          
        case  SPURTL:    //һ�壨��ͨ��̣�
               
             TIM4->CCR3 = 600;delay(1000000); delay(1000000); delay(1000000);//����ռ�ձȣ���ʱ
             StartGet();
             wallget();
             goalwall();
             
             mouseSpurt_CC();
             
             onestep();
             onestep();
             mouseTurnback();
             objectGoTo1(GucXStart,GucYStart);
             //onestep3();
             onestep3();
             onestep1();
             onestep();
             mouseTurnbackqidian();
           
          // mouseStop(); while(1);
          GucMouseTask = SPURT45; 
            break; 
            
        case SPURT45:
            
             TIM4->CCR3 = 600;
             mouseSpurt_45();
             usepiancha=1;
             onestep1();

             mouseTurnback();
             //onestep3();

             __GmSPID.sRef=145;  
            // onestep2();
             TIM4->CCR3 = 0;
             
             objectGoTo1(GucXStart,GucYStart);
             
             mouseTurnbackqi(); 
  
             while (1) 
            {
                if (startCheck() == true)
                {
                    break;
                }
            }
            break;                 
        default:
            break;
        }
    }
}






     
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
