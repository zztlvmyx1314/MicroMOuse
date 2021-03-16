#include"iic.h"

void Delay (uint32 Time)						   // 1us
{
    uint32 i;
   
    while (Time--) {
        for (i = 0; i < 7; i++);
    }
}

void __Delay (uint32 Time)						   // 1us
{
    uint32 i;
   
    while (Time--) {
        for (i = 0; i < 1; i++);
    }
}

void IIC_Init(void)
{					     
    GPIOB->CRL&=0X00FFFFFF;
    GPIOB->CRL|=0X33000000;	 
    IIC_SCL=1;
    IIC_SDA=1;

}

void I2Cstart(void)
{ 
	SDA_OUT(); 
	IIC_SCL = 0;
	IIC_SDA = 1;
  	Delay(5);//5us
	IIC_SCL = 1;
  	Delay(5);
	IIC_SDA = 0;
	Delay(5);//5us
  	IIC_SCL = 0;
	Delay(5);
}

void I2Cstop(void)
{ 
	SDA_OUT();
	IIC_SDA = 0;
	IIC_SCL = 0;
  	Delay(5);
  	IIC_SCL = 1;
  	Delay(5);
  	IIC_SDA = 1;
  	Delay(5);
	IIC_SCL = 0;
	Delay(5);
}

void I2CACK(void)
{ 	
  	SDA_OUT();
  	IIC_SDA = 0;
  	IIC_SCL = 0;
	Delay(5);
	IIC_SCL = 1;
  	Delay(5);
  	IIC_SCL = 0;
	Delay(5);
}

void I2CNACK(void)
{
 	SDA_OUT();
	Delay(5);
  	IIC_SDA = 1;
  	IIC_SCL = 0;
	Delay(5);
	IIC_SCL = 1;
  	Delay(5);
  	IIC_SCL = 0;
	Delay(5);
}

uint32 I2CCACK(void)
{ 
	SDA_IN();
	IIC_SCL = 0;
	Delay(5);
  	IIC_SCL = 1;
  	return(READ_SDA);
}

void send_fm24LC16(uint32 data)
{ 
	uint32 flag;
	uint32 sz;
  	SDA_OUT();
	IIC_SDA = 0;
	IIC_SCL = 0;
	Delay(5);
  	for(flag=0x0080;flag!=0x00;flag=flag/2)
  	{  
   		sz = data & flag;
 		if(sz==0)
                { IIC_SDA = 0;}
    	        else  
                { IIC_SDA = 1;}
		Delay(5);
		IIC_SCL = 1;
		Delay(5);
                IIC_SCL = 0;
		IIC_SDA = 0;
  	}
}

//д�����ӳ���
uint32 write_fm24LC16(uint32 *array,uint32 address,uint32 control,uint32 len)
{ 
	uint32 dat;
  	I2Cstart();
	dat=control;
  	send_fm24LC16(dat);
  	dat=I2CCACK();
  	if(dat==0)
  	{
		dat = address;
   		send_fm24LC16(dat);
   		dat = I2CCACK();
   		if(dat==0)
    	        { 
		      for(;len!=0;len--,array++)
      		      {
			send_fm24LC16(*array);
       			dat = I2CCACK();
       			if(dat==1)
                          break;
      		      }
    
 		}
  	}
    I2Cstop();
    return(dat);
} 

//��I2C����
uint32 receive_fm24LC16(void)
{ 
	uint32 sz=0;
	uint32 i;
   	SDA_IN();
   	IIC_SCL = 0;
   	Delay(5);
  	for(i=0;i<8;i++)
   	{  
		if(READ_SDA)
      	        {sz|=0x01;}
		Delay(3);
		IIC_SCL = 1;
      	        Delay(5);
		IIC_SCL = 0;
	  	if(i<7)
	  	{sz=sz<<1;}
		Delay(5);
   	}
   	return(sz);
}

//�������ӳ���
uint32 read_fm24LC16(uint32 *array,uint32 address,uint32 control,uint32 len)
{ 
	uint32 dat;
  	I2Cstart();
  	dat=(control & 0xfe);
	send_fm24LC16(dat);
	IIC_SDA = 0;
	IIC_SCL = 0;
	Delay(5);
  	dat=I2CCACK();
  	if(dat==0)
  	{  
	dat=address;
     	send_fm24LC16(dat);
     	dat=I2CCACK();
     	if(dat==0)
     	{    
	I2Cstart();
	dat=control;
        send_fm24LC16(dat); 
        dat=I2CCACK();
        if(dat==0)
         {  
	for(;len!=0;len--,array++)
          { 
	dat=receive_fm24LC16();
	I2CACK();
         *array=dat;
      	}
       	I2CNACK();
        I2Cstop();
         dat=0;
            }
       	}
    }     
   	return(dat);
}

void IIC_Start(void)
{
      SDA_OUT(); 
      IIC_SCL = 0;
      IIC_SDA = 1;
       __Delay (2);
      IIC_SCL = 1;
       __Delay (2);
      IIC_SDA = 0;
       __Delay (2);
      IIC_SCL = 0;
       __Delay (2);
}

void IIC_Stop(void)
{ 
      SDA_OUT();
      IIC_SDA = 0;
      IIC_SCL = 0;
       __Delay (2);
      IIC_SCL = 1;
       __Delay (2);
      IIC_SDA = 1;
       __Delay (2);
      IIC_SCL = 0;
       __Delay (2);
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8 IIC_Wait_Ack(void)
{
    uint8 ucErrTime=0;
    SDA_IN();      //SDA����Ϊ����  
    IIC_SDA = 1;__Delay (2);	   
    IIC_SCL = 1;__Delay (2); 
    while(READ_SDA)
    {
        ucErrTime++;
	if(ucErrTime>250)
	{
	    IIC_Stop();
	    return 1;
	}
    }
    IIC_SCL = 0;   
    return 0;  
}

void IIC_Ack(void)
{ 	
  	SDA_OUT();
  	IIC_SDA = 0;
  	IIC_SCL = 0;
	__Delay (2); 
	IIC_SCL = 1;
  	__Delay (2); 
  	IIC_SCL = 0;
	__Delay (2); 
}
//������ACKӦ��		    
void IIC_NAck(void)
{
 	SDA_OUT();
	__Delay (2); 
  	IIC_SDA = 0;
  	IIC_SCL = 0;
	__Delay (2); 
	IIC_SCL = 1;
  	__Delay (2); 
  	IIC_SCL = 0;
	__Delay (2); 
}

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8 txd)
{                        
    uint8 t;   
    SDA_OUT(); 	    
    IIC_SCL = 0;
    for(t=0;t<8;t++)
    {   
      if((txd&0x80)>>7)
      {
          IIC_SDA = 1;
      }
      else
      {
          IIC_SDA = 0;
      }
        txd<<=1; 	  
        __Delay (2);   
       IIC_SCL = 1;
        __Delay (2);
       IIC_SCL = 0;	
        __Delay (2);
    }	 
}

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
        for(i=0;i<8;i++ )
	{
        IIC_SCL = 0; 
        __Delay (1);
        IIC_SCL = 1;
        receive<<=1;
        if(READ_SDA)receive++;   
	__Delay (1); 
  }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

uint8 MPU6050_ReadI2C(uint8 REG_Address)
{
	uint8 REG_data;
	IIC_Start();                  //��ʼ�ź�
	IIC_Send_Byte(SlaveAddress);  //�����豸��ַ+д�ź�
	REG_data=IIC_Wait_Ack();	   
	IIC_Send_Byte(REG_Address);   //���ʹ洢��Ԫ��ַ����0��ʼ
	REG_data=IIC_Wait_Ack();	   
	IIC_Start();                  //��ʼ�ź�
	IIC_Send_Byte(SlaveAddress+1);//�����豸��ַ+���ź�
	REG_data=IIC_Wait_Ack();	   
        REG_data=IIC_Read_Byte(0);		//��ȡһ���ֽ�,�������ٶ�,����NAK,�����Ĵ�������
	IIC_Stop();	                  //ֹͣ�ź�
	return REG_data;
}

void MPU6050_WriteI2C(uint8 REG_Address,uint8 REG_data)
{
    IIC_Start();                   //��ʼ�ź�
    IIC_Send_Byte(SlaveAddress);   //�����豸��ַ+д�ź�
    IIC_Wait_Ack();	   
    IIC_Send_Byte(REG_Address);    //�ڲ��Ĵ�����ַ
    IIC_Wait_Ack(); 	 										  		   
    IIC_Send_Byte(REG_data);       //�ڲ��Ĵ�������
    IIC_Wait_Ack(); 	 										  		   
    IIC_Stop();                    //����ֹͣ�ź�
}

void MPU6050_Init(void)
{
    MPU6050_WriteI2C(PWR_MGMT_1, 0x00);	//�������״̬
    MPU6050_WriteI2C(SMPLRT_DIV, 0x07);
    MPU6050_WriteI2C(CONFIG, 0x06);
    MPU6050_WriteI2C(GYRO_CONFIG, 0x18);
}

uint16 GetData(uint8 REG_Address)
{
    uint8 H,L;
    H=MPU6050_ReadI2C(REG_Address);
    L=MPU6050_ReadI2C(REG_Address+1);
    return (H<<8)+L; 
}

