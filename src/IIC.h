#ifndef __MYIIC_H__
#define __MYIIC_H__

#include "gpio.h"
#include "common.h"
    		   
//IO方向设置
#define SDA_IN()  {PTC->PDDR &= ~(1 << 11);}
#define SDA_OUT() {PTC->PDDR |= (1 << 11);}

//IO操作函数	 
#define IIC_SCL    PCout(10) //SCL
#define IIC_SDA    PCout(11) //SDA	 
#define READ_SDA   PCin(11)  //输入SDA 

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
void Delay(uint32_t time);

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  

#endif


