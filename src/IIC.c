#include "IIC.h"
#include "gpio.h"
#include "common.h"

void Delay(uint32_t time)
{
  while(time--);
}

//��ʼ��IIC
void IIC_Init(void)
{					     
// 	RCC->APB2ENR|=1<<4;//��ʹ������IO PORTCʱ�� 							 
//	GPIOC->CRH&=0XFFF00FFF;//PC11/12 �������
//	GPIOC->CRH|=0X00033000;	   
//	GPIOC->ODR|=3<<11;     //PC11,12 �����
	GPIO_QuickInit(HW_GPIOE, 24, kGPIO_Mode_OPP);	
	GPIO_QuickInit(HW_GPIOE, 25, kGPIO_Mode_OPP);
	IIC_SDA=1;	  	  
	IIC_SCL=1;
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	Delay(2);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	Delay(2);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	Delay(2);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	Delay(4); 							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
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
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay(1);      //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		Delay(2); 
		IIC_SCL=0;	
		Delay(4); 
    }
	IIC_Wait_Ack();	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}
