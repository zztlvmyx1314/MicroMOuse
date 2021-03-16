#include "stmflash.h"


volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;   //Flash����״̬����

u16 STMFLASH_ReadHalfWord(u32 addr)
{
	return *(u16 *)addr;
}


#if STM32_FLASH_WREN
	
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
	u16 i;
	
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
		WriteAddr+=2;
	}
}

#if STM32_FLASH_SIZE<256
	#define STM_SECTOR_SIZE 1024
#else
	#define STM_SECTOR_SIZE 2048
#endif

u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
	u32 secpos;     //������ַ
	u16 secoff;     //������ƫ�Ƶ�ַ��16λ�ּ��㣩
	u16 secremain;   //������ʣ���ַ��16λ�ּ��㣩
	
	u16 i;
	u32 offaddr;   //ȥ��0x08000000��ĵ�ַ
	
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr>=STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)
		return ;  //�Ƿ���ַ
	
	FLASH_Unlock();     //����
	
	offaddr=WriteAddr-STM32_FLASH_BASE;
	secpos=offaddr/STM_SECTOR_SIZE;
	secoff=(offaddr%STM_SECTOR_SIZE)/2;
	secremain=STM_SECTOR_SIZE/2-secoff;
	
	if(NumToWrite<=secremain)
		secremain=NumToWrite;
	
	while(1)
	{
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);
		
		for(i=0;i<secremain;i++)
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)
				break;
		}
		
		if(i<secremain)
		{
				FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//�����������
				for(i=0;i<secremain;i++)//����
				{
					STMFLASH_BUF[i+secoff]=pBuffer[i];	  
				}
				 STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);
		}
		else 
				STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������ 	
			
		if(NumToWrite==secremain)
				break;//д�������
		else
		{
				secpos++;				//������ַ��1
				secoff=0;				//ƫ��λ��Ϊ0 	 
				pBuffer+=secremain;  	//ָ��ƫ��
				WriteAddr+=secremain;	//д��ַƫ��	   
				NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
				if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//��һ����������д����
				else secremain=NumToWrite;//��һ����������д����
		}				
	}	
	FLASH_Lock();
}

#endif


void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);
		
		ReadAddr+=2;
		
	}
}
	

void Test_Write(u32 WriteAddr,u16 WriteData)
{
	STMFLASH_Write(WriteAddr,&WriteData,1);
}



void STMFLASH_WriteOneWord(u32 WriteAddr,u32 WriteData)
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
	
	FLASHStatus=FLASH_ErasePage(STARTADDR);
	
	if(FLASHStatus)
	{
		FLASHStatus=FLASH_ProgramWord(STARTADDR+WriteAddr,WriteData);
	}
	FLASH_Lock();
}

/****************************************************************
*Name:		ReadFlashNBtye
*Function:	���ڲ�Flash��ȡN�ֽ�����
*Input:		ReadAddress�����ݵ�ַ��ƫ�Ƶ�ַ��ReadBuf������ָ��	ReadNum����ȡ�ֽ���
*Output:	��ȡ���ֽ���  
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:		
****************************************************************/
int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum) 
{
		int DataNum = 0;
		ReadAddress = (uint32_t)STARTADDR + ReadAddress; 
		while(DataNum < ReadNum) 
		{
			 *(ReadBuf + DataNum) = *(__IO uint8_t*) ReadAddress++;
			 DataNum++;
		}
		return DataNum;
}
