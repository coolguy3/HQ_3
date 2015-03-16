#ifndef __MYIIC_H
#define __MYIIC_H
#include "common.h"
    		   
//IO��������
#define SDA_IN()  {PTE->PDDR &= ~(1 << 24);}
#define SDA_OUT() {PTE->PDDR |= (1 << 24);}

//IO��������	 
#define IIC_SCL    PEout(25) //SCL
#define IIC_SDA    PEout(24) //SDA	 
#define READ_SDA   PEin(24)  //����SDA 

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
void Delay(uint32_t time);

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  

#endif
















