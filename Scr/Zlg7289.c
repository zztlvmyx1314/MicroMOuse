#include "zlg7289.h"

/*********************************************************************************************************
** Function name:       ZLG7289_Delay
** Descriptions:        ��ʱN��΢��
** input parameters:    x: ��ʱʱ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void ZLG7289_Delay(uint16 x)
{	
	x += x;
	while(x--);
}

/*********************************************************************************************************
** Function name:       ZLG7289_DIO_OUT
** Descriptions:        ��ZLG728_DI0���ó����ģʽ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void ZLG7289_DIO_OUT(void)
{
	//DIO=1;
    GPIOB->CRL&=0XFF0FFFFF; 
    GPIOB->CRL|=0X00300000;    //PD12������� 
   // GPIOD->ODR|=1<<12;          //PD12�����	
}
/*********************************************************************************************************
** Function name:       ZLG7289_DIO_IN
** Descriptions:        ��ZLG728_DI0���ó�����ģʽ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void ZLG7289_DIO_IN(void)
{
    GPIOB->CRL&=0XFF0FFFFF;
    GPIOB->CRL|=0X00400000;    //PD12��������	
}

/*********************************************************************************************************
** Function name:       __zlg7289SPIWrite
** Descriptions:        ��SPI ����д��1 ���ֽڵ����ݡ�
** input parameters:    cDat��Ҫд�������
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __zlg7289SPIWrite (char cDat)
{
      char CT=8;
      ZLG7289_DIO_OUT();	 //DIO���ó����ģʽ
      do{
           if((cDat & 0x80) == 0x80)
            {
                 ZLG7289_DIO=1;	
            }
            else
            {
                 ZLG7289_DIO=0;
            }
            cDat <<= 1;
            ZLG7289_CLK=1;
            ZLG7289_Delay(50);
            ZLG7289_CLK=0;
            ZLG7289_Delay(50);
        }while(--CT!=0);
}
/*********************************************************************************************************
** Function name:       __zlg7289SPIRead
** Descriptions:        ��SPI ���߶�ȡ1 ���ֽڵ����ݡ�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��ȡ��������
*********************************************************************************************************/
 

char __zlg7289SPIRead (void)
{
    char cDat = 0;
    char cT   = 8;
    ZLG7289_DIO_IN();   
  
   do
	{
	    ZLG7289_CLK=1;
	    ZLG7289_Delay(50);
	    cDat<<=1;  
	    if(PBin(5)==1)  cDat++;
	    ZLG7289_CLK=0;
	    ZLG7289_Delay(50);

	} while ( --cT != 0 );
	return 	cDat;
} 

/*********************************************************************************************************
** Function name:       zlg7289Cmd
** Descriptions:        ִ��ZLG7289 ��ָ�
** input parameters:    cCmd��������
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void zlg7289Cmd (char  cCmd)
{
    ZLG7289_CS=0;
    ZLG7289_Delay(200);
    __zlg7289SPIWrite (cCmd);
    ZLG7289_CS=1;
    ZLG7289_Delay(50);	
} 
/*********************************************************************************************************
** Function name:       zlg7289CmdDat
** Descriptions:        ִ��ZLG7289 ������ָ�
** input parameters:    cCmd��������
**                      cDat������
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void zlg7289CmdDat (uint8  cCmd, char  cDat)
{
    ZLG7289_CS=0;
    ZLG7289_Delay(200);
    __zlg7289SPIWrite (cCmd);
    ZLG7289_Delay(80);
    __zlg7289SPIWrite (cDat);
    ZLG7289_CS=1;
    ZLG7289_Delay(50);
}
/*********************************************************************************************************
** Function name:       zlg7289Download
** Descriptions:        �������ݡ�
** input parameters:    ucMod=0�� ���������Ұ���ʽ0 ����
**                      ucMod=1�� ���������Ұ���ʽ1 ����
**                      ucMod=2�� �������ݵ�������
**                      cX��      ����ܱ�ţ������꣩��ȡֵ0��7
**                      cDp=0��   С���㲻��
**                      cDp=1��   С������
**                      cDat��    Ҫ��ʾ������
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void zlg7289Download (u8  ucMod, char  cX, char  cDp, char  cDat)
{
    unsigned char ucModDat[3] = {0x80,0xC8,0x90};
    unsigned char ucD1;
    unsigned char ucD2;
    
    if (ucMod > 2) {
        ucMod = 2;
    }
    
    ucD1  = ucModDat[ucMod];
    cX   &= 0x07;
    ucD1 |= cX;
    ucD2  = cDat & 0x7F;
    
    if (cDp  == 1) {
        ucD2 |= 0x80;
    }
    zlg7289CmdDat(ucD1, ucD2);
}
/*********************************************************************************************************
** Function name:       zlg7289Key
** Descriptions:        ִ��ZLG7289 �������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ���ض����İ���ֵ��0��63���������0xFF ���ʾû�м�����
*********************************************************************************************************/
char zlg7289Key (void)
{
   char key;
   
   ZLG7289_CS=0;
   ZLG7289_Delay(200);
   __zlg7289SPIWrite(0x15);
   ZLG7289_Delay(100);
   key=__zlg7289SPIRead();
   ZLG7289_CS=1;
   ZLG7289_Delay(50);

   return key;
}
/*********************************************************************************************************
** Function name:       __zlg7289KeyInit
** Descriptions:        ��ZLG7289�����жϳ�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __zlg7289KeyInit (void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	   //�����ж����ţ���������
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 		  //����2λ�����ȼ�����ռ���ȼ�����2λ�����ȼ�����Ӧ���ȼ���
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;             
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 	  //�����ж���ռ���ȵȼ�Ϊ�ڶ�����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 	          //�����ж������ȵȼ�Ϊ�ڶ�����
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);    
 
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource3);  //ѡ���ж�����
    EXTI_InitStructure.EXTI_Line = EXTI_Line3 ;		
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			//�ж�ģʽ�����¼�
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//�½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);    
    
}
/*********************************************************************************************************
** Function name:       ZLG7289Init
** Descriptions:        ��ZLG7289���г�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void ZLG7289Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_5 |GPIO_Pin_4;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    ZLG7289_DIO=1;
    ZLG7289_CLK=0;	
    ZLG7289_CS=1;
    __zlg7289KeyInit ();
    zlg7289Reset();	
}




