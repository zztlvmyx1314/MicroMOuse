/****************************************Copyright (c)****************************************************
**                               
**                                     
**                                 
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           IIC.h
** Last modified Date:  2013/11/20
** Last Version:        V1.00
** Description:         AT24C02���STM32����������ͷ�ļ�
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          �½ܺ�
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
#ifndef _iic_H_
#define _iic_H_
#include "stm32f10x.h"
#include "BitBand.h"
#include "Type.h"
// ����MPU6050�ڲ���ַ
//****************************************
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_XOUT_H	0x3B	//0
#define	ACCEL_XOUT_L	0x3C	//1
#define	ACCEL_YOUT_H	0x3D	//2
#define	ACCEL_YOUT_L	0x3E	//3     
#define	ACCEL_ZOUT_H	0x3F	//4
#define	ACCEL_ZOUT_L	0x40	//5
#define	TEMP_OUT_H		0x41	//6
#define	TEMP_OUT_L		0x42	//7
#define	GYRO_XOUT_H		0x43	//8
#define	GYRO_XOUT_L		0x44	//9
#define	GYRO_YOUT_H		0x45	//10
#define	GYRO_YOUT_L		0x46	//11
#define	GYRO_ZOUT_H		0x47  //12
#define	GYRO_ZOUT_L		0x48	//13

#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	SlaveAddress	0xD0	//IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ
/*********************************************************************************************************
  ZLG7289�˿ڶ���
*********************************************************************************************************/       

#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=8<<28;}
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=3<<28;}

 
#define IIC_SCL    PBout(6) 
#define IIC_SDA    PBout(7)  
#define READ_SDA   PBin(7)  
	 
extern void IIC_Init(void);
extern u32 read_fm24LC16(u32 *array,u32 address,u32 control,u32 len);
extern u32 write_fm24LC16(u32 *array,u32 address,u32 control,u32 len);
uint16 GetData(uint8 REG_Address);
void MPU6050_Init(void);
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
