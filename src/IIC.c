#include "IIC.h"
#include "gpio.h"
#include "common.h"

void Delay(uint32_t time)
{
  while(time--);
}

//初始化IIC
void IIC_Init(void)
{					     
// 	RCC->APB2ENR|=1<<4;//先使能外设IO PORTC时钟 							 
//	GPIOC->CRH&=0XFFF00FFF;//PC11/12 推挽输出
//	GPIOC->CRH|=0X00033000;	   
//	GPIOC->ODR|=3<<11;     //PC11,12 输出高
	GPIO_QuickInit(HW_GPIOE, 24, kGPIO_Mode_OPP);	
	GPIO_QuickInit(HW_GPIOE, 25, kGPIO_Mode_OPP);
	IIC_SDA=1;	  	  
	IIC_SCL=1;
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	Delay(2);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	Delay(2);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	Delay(2);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	Delay(4); 							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
    Delay(2);	   
	IIC_SCL=1; 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	Delay(2);
	IIC_SCL=1;
	Delay(2); 
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	Delay(2);
	IIC_SCL=1;
	Delay(2); 
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay(1);      //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		Delay(2); 
		IIC_SCL=0;	
		Delay(4); 
    }
	IIC_Wait_Ack();	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        Delay(4); 
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		Delay(2); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}
