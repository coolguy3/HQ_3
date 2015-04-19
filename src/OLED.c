#include "OLED.h" 
#include "ZIKU.h"

uint8_t OLED_GRAM[128][8];
enum Key { Key_Up,Key_Down,Key_Left,Key_Right,Key_Enter,No_Key	} Key;
struct Parameter						//FLASH保存的参数，每次初始化后都从FLASH读出保存的参数，调整后用按键Enter键保存
{
	float Stand_Kp,Stand_Kd;
	float Speed_Kp,Speed_Ki,Speed_Kd;
}Flash_Parameter;
uint8_t Motor_Set_Flag = 0;	//按键Enter键控制，是否开启电机控制

void OLED_Delay(uint32_t time)
{
  while(time--);
}

//初始化IIC
void OLED_IIC_Init(void)
{					     
	GPIO_QuickInit(HW_GPIOB, 0, kGPIO_Mode_OPP);	
	GPIO_QuickInit(HW_GPIOB, 1, kGPIO_Mode_OPP);
	OLED_SDA=1;	  	  
	OLED_SCL=1;
}

//产生OLED_IIC起始信号
void OLED_IIC_Start(void)
{
	OLED_OUT();     //sda线输出
	OLED_SDA=1;	  	  
	OLED_SCL=1;
	OLED_Delay(2);
 	OLED_SDA=0;//START:when CLK is high,DATA change form high to low 
	OLED_Delay(2);
	OLED_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  

//产生OLED_IIC停止信号
void OLED_IIC_Stop(void)
{
	OLED_OUT();//sda线输出
	OLED_SCL=0;
	OLED_SDA=0;//STOP:when CLK is high DATA change form low to high
 	OLED_Delay(2);
	OLED_SCL=1; 
	OLED_SDA=1;//发送OLED_I2C总线结束信号
	OLED_Delay(4); 							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t OLED_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	OLED_IN();      //OLED_SDA设置为输入  
	OLED_Delay(2);	   
	OLED_SCL=1; 
	while(READ_OLED_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			OLED_IIC_Stop();
			return 1;
		}
	}
	OLED_SCL=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void OLED_IIC_Ack(void)
{
	OLED_SCL=0;
	OLED_OUT();
	OLED_SDA=0;
	OLED_Delay(2);
	OLED_SCL=1;
	OLED_Delay(2); 
	OLED_SCL=0;
}

//不产生ACK应答		    
void OLED_IIC_NAck(void)
{
	OLED_SCL=0;
	OLED_OUT();
	OLED_SDA=1;
	OLED_Delay(2);
	OLED_SCL=1;
	OLED_Delay(2); 
	OLED_SCL=0;
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void OLED_IIC_Send_Byte(uint8_t txd)
{                        
	uint8_t t;   
	OLED_OUT(); 	    
	OLED_SCL=0;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
		OLED_SDA=(txd&0x80)>>7;
		txd<<=1; 	  
		OLED_Delay(1);      //对TEA5767这三个延时都是必须的
		OLED_SCL=1;
		OLED_Delay(2); 
		OLED_SCL=0;	
		OLED_Delay(4); 
	}
	OLED_IIC_Wait_Ack();	 
} 

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t OLED_IIC_Read_Byte(uint8_t ack)
{
	uint8_t i,receive=0;
	OLED_IN();//OLED_SDA设置为输入
	for(i=0;i<8;i++ )
	{
		OLED_SCL=0; 
		OLED_Delay(4); 
		OLED_SCL=1;
		receive<<=1;
		if(READ_OLED_SDA)receive++;   
		OLED_Delay(2); 
	}					 
	if (!ack)
		OLED_IIC_NAck();//发送nACK
	else
		OLED_IIC_Ack(); //发送ACK   
	return receive;
}

//OLED写命令
void OLED_WrCmd(uint8_t IIC_Command)
{
	OLED_IIC_Start();
	OLED_IIC_Send_Byte(0x78);            //Slave address,SA0=0
	OLED_IIC_Send_Byte(0x00);			//write command
	OLED_IIC_Send_Byte(IIC_Command);
	OLED_IIC_Stop();
}

//OLED写数据
void OLED_WrDat(uint8_t IIC_Data)
{
	OLED_IIC_Start();
	OLED_IIC_Send_Byte(0x78);
	OLED_IIC_Send_Byte(0x40);			//write data
	OLED_IIC_Send_Byte(IIC_Data);
	OLED_IIC_Stop();
}

//OLED设置坐标
void OLED_Set_Pos(uint8_t x, uint8_t y) 
{ 
	OLED_WrCmd(0xb0+y);
	OLED_WrCmd(((x&0xf0)>>4)|0x10);
	OLED_WrCmd((x&0x0f)|0x01);
} 

//OLED全屏
void OLED_Fill(uint8_t bmp_dat) 
{
	uint8_t y,x;
	for(y=0;y<8;y++)
	{
		OLED_WrCmd(0xb0+y);
		OLED_WrCmd(0x01);
		OLED_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
		OLED_WrDat(bmp_dat);
	}
}

//OLED复位
void OLED_CLS(void)
{
	uint8_t y,x;
	for(y=0;y<8;y++)
	{
		OLED_WrCmd(0xb0+y);
		OLED_WrCmd(0x01);
		OLED_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
		OLED_WrDat(0);
	}
}

//初始化OLED
void OLED_Init(void)
{
	OLED_IIC_Init();
	OLED_Delay(33333);
	OLED_WrCmd(0xae);//--turn off oled panel
	OLED_WrCmd(0x00);//---set low column address
	OLED_WrCmd(0x10);//---set high column address
	OLED_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WrCmd(0x81);//--set contrast control register
	OLED_WrCmd(Brightness); // Set SEG Output Current Brightness
	OLED_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	OLED_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	OLED_WrCmd(0xa6);//--set normal display
	OLED_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
	OLED_WrCmd(0x3f);//--1/64 duty
	OLED_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WrCmd(0x00);//-not offset
	OLED_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
	OLED_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WrCmd(0xd9);//--set pre-charge period
	OLED_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WrCmd(0xda);//--set com pins hardware configuration
	OLED_WrCmd(0x12);
	OLED_WrCmd(0xdb);//--set vcomh
	OLED_WrCmd(0x40);//Set VCOM Deselect Level
	OLED_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WrCmd(0x02);//
	OLED_WrCmd(0x8d);//--set Charge Pump enable/disable
	OLED_WrCmd(0x14);//--set(0x10) disable
	OLED_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
	OLED_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7) 
	OLED_WrCmd(0xaf);//--turn on oled panel
	OLED_Fill(0x00); //初始清屏

}

//显示6*8一个ASCII字符	显示的坐标（x,y），y为页范围0～7
void OLED_P6x8Char(uint8_t x,uint8_t y,uint8_t dat)
{
  uint8_t c=0,i=0;         
  c =dat-32;
  if(x>126){x=0;y++;}
  OLED_Set_Pos(x,y);    
	for(i=0;i<6;i++)     
	  OLED_WrDat(F6x8[c][i]);
	//Delayms(1);
}

//显示6*8一组标准ASCII字符串	显示的坐标（x,y），y为页范围0～7
void OLED_P6x8Str(uint8_t x, uint8_t y,char ch[])
{
	uint8_t c=0,i=0,j=0;
	while (ch[j]!='\0')
	{
		c =ch[j]-32;
		if(x>126){x=0;y++;}
		OLED_Set_Pos(x,y);
		for(i=0;i<6;i++)
		OLED_WrDat(F6x8[c][i]);
		x+=6;
		j++;
	}
}

//显示8*16一组标准ASCII字符串	 显示的坐标（x,y），y为页范围0～7
void OLED_P8x16Str(uint8_t x,uint8_t y,char ch[])
{
	uint8_t c=0,i=0,j=0;
	while (ch[j]!='\0')
	{
		c =ch[j]-32;
		if(x>120){x=0;y++;}
		OLED_Set_Pos(x,y);
		for(i=0;i<8;i++)
		OLED_WrDat(F8X16[c*16+i]);
		OLED_Set_Pos(x,y+1);
		for(i=0;i<8;i++)
		OLED_WrDat(F8X16[c*16+i+8]);
		x+=8;
		j++;
	}
}

//画点
void OLED_PutPixel(uint8_t x,uint8_t y)
{
	uint8_t data1;  
	 
  OLED_Set_Pos(x,y); 
	data1 = 0x01<<(y%8); 	
	OLED_WrCmd(0xb0+(y>>3));
	OLED_WrCmd(((x&0xf0)>>4)|0x10);
	OLED_WrCmd((x&0x0f)|0x00);
	OLED_WrDat(data1); 	 	
}

//功能描述：显示16*16点阵  显示的坐标（x,y），y为页范围0～7
void OLED_P16x16Ch(uint8_t x, uint8_t y, uint8_t N)
{
	uint8_t wm=0;
	uint32_t adder=32*N;
	OLED_Set_Pos(x , y);
	for(wm = 0;wm < 16;wm++)
	{
		OLED_WrDat(F16x16[adder]);
		adder += 1;
	}
	OLED_Set_Pos(x,y + 1);
	for(wm = 0;wm < 16;wm++)
	{
		OLED_WrDat(F16x16[adder]);
		adder += 1;
	} 	  	
}

//功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7
void Draw_BMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,uint8_t BMP[])
{
	uint32_t j=0;
	uint8_t x,y;

  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
	    {      
	    	OLED_WrDat(BMP[j++]);
	    }
	}
}

//功能描述：显示5位有效数字的整型数据  显示的坐标（x,y），x: 0 - 9	y: 0 - 2
void OLED_Show_Data(uint8_t x,uint8_t y,int32_t var)
{
	if(var < 0)
	{
		var = -var;
		OLED_P6x8Char(x*6,y,'-');
	}   
	else
	{  
		OLED_P6x8Char(x*6,y,' ');
	}
	
	OLED_P6x8Char((1+x)*6,y,var%100000/10000 + 48);
	OLED_P6x8Char((2+x)*6,y,var%10000/1000 + 48);
	OLED_P6x8Char((3+x)*6,y,var%1000/100 + 48);
	OLED_P6x8Char((4+x)*6,y,var%100/10 + 48);
	OLED_P6x8Char((5+x)*6,y,var%10 + 48);
}

//功能描述：显示一位小数的百位数浮点型数据  显示的坐标（x,y），x: 0 - 9	y: 0 - 2
void OLED_Show_Float(uint8_t x,uint8_t y,float var)
{
	if(var < 0)
	{
		var = -var;
		OLED_P6x8Char(x*6,y,'-');
	}   
	else
	{  
		OLED_P6x8Char(x*6,y,' ');
	}
	
	if((int16_t)(var+0.05) / 100 != 0)
	{
		OLED_P6x8Char((1+x)*6,y,((int16_t)((var+0.05)*10.0))%10000/1000 + 48);
		OLED_P6x8Char((2+x)*6,y,((int16_t)((var+0.05)*10.0))%1000/100 + 48);
		OLED_P6x8Char((3+x)*6,y,((int16_t)((var+0.05)*10.0))%100/10 + 48);
		OLED_P6x8Char((4+x)*6,y,46);
		OLED_P6x8Char((5+x)*6,y,((int16_t)((var+0.05)*10.0))%10 + 48);
	}
	else	if((int16_t)(var+0.05) / 10 != 0)
	{
		OLED_P6x8Char((1+x)*6,y,((int16_t)((var+0.05)*10.0))%1000/100 + 48);
		OLED_P6x8Char((2+x)*6,y,((int16_t)((var+0.05)*10.0))%100/10 + 48);
		OLED_P6x8Char((3+x)*6,y,46);
		OLED_P6x8Char((4+x)*6,y,((int16_t)((var+0.05)*10.0))%10 + 48);
	}
	else
	{
		OLED_P6x8Char((1+x)*6,y,((int16_t)((var+0.05)*10.0))%100/10  + 48);
		OLED_P6x8Char((2+x)*6,y,46);
		OLED_P6x8Char((3+x)*6,y,((int16_t)((var+0.05)*10.0))%10 + 48);		
	}
}

int MKP512FlashInit(void)
{
    uint32_t clock;
    uint32_t flash_clock = CLOCK_GetClockFrequency(kFlashClock, &clock);
    /* fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify) */
		
		Flash_Parameter = * ( (struct Parameter *)TEST_ADDR_BEIGN );	//读回FLASH保存的参数

    /* func:Init is SSD API */    
    return Init(0x00000000, clock, 2); 
}

void Motor_Test()
{
	uint16_t Duty1 = 5000 , Duty2 = 5000;
	uint8_t Test_Finished = 0;
	while(1)
	{
				
		Key = (enum Key)Key_Scan(!DIP_1);
		switch(Key)
		{
			case Key_Up			:	Duty1 += 100;				// 0-10000 对应占空比 0-100%
												if(Duty1 > 10000)	Duty1 = 10000;
												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , Duty1);		
												OLED_Show_Data(0,0,Duty1);
												break;	
			case Key_Down		: Duty1 -= 100;
												if(Duty1 < 0)			Duty1 = 0;			
												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , Duty1);		
												OLED_Show_Data(0,0,Duty1);
												break;	//右轮
			case Key_Left		: Duty2 += 100;	
												if(Duty2 > 10000)	Duty2 = 10000;
												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4 , Duty2);		
												OLED_Show_Data(0,1,Duty2);
												break;	//左轮
			case Key_Right	: Duty2 -= 100;	
												if(Duty2 < 0)			Duty2 = 0;			
												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4 , Duty2);		
												OLED_Show_Data(0,1,Duty2);
												break;
			case Key_Enter	: Test_Finished = 1;	
												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , 5000);
												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4 , 5000);
												break;
			default :  					DelayMs(10);				break;	
		}
		
		if(Test_Finished)
		{
			break;
		}
		
	}
}

	
void OLED_UI()
{
	extern struct Quad_PID PID_Stand , PID_Speed;

	static int8_t OLED_Show = 0;				//OLED显示的界面号
	static float * OLED_Adjust = 0;		//存放需要OLED调整的参数的指针
	float Null = 0;
	
	Key = (enum Key)Key_Scan(!DIP_1);	//第一个拨码开关---0,不支持连续按;1,支持连续按
	switch(Key)
	{
		case Key_Up			:	*OLED_Adjust += 0.1;			break;	
		case Key_Down		: *OLED_Adjust -= 0.1;			break;	
		case Key_Left		: OLED_Show--;	OLED_Fill(0x00);	if(OLED_Show < 0)OLED_Show = 0;			break;	
		case Key_Right	: OLED_Show++;	OLED_Fill(0x00);	if(OLED_Show > 6)OLED_Show = 0;			break;
		case Key_Enter	:	EraseSector(TEST_ADDR_BEIGN);
											ProgramPage(TEST_ADDR_BEIGN, sizeof(Flash_Parameter), (void*)&Flash_Parameter);		//保存参数到FLASH
											pidInit(&PID_Stand, Flash_Parameter.Stand_Kp , 0 , Flash_Parameter.Stand_Kd , 44);	//REset pid各值，并更新参数
											pidInit(&PID_Speed, Flash_Parameter.Speed_Kp , Flash_Parameter.Speed_Ki , Flash_Parameter.Speed_Kd , 0);
											Motor_Set_Flag = 1;
											break;
		default : DelayMs(20);	 break;	
	}
	
	switch(OLED_Show)
	{
		case 0	:		OLED_P6x8Str(0,0,"Stand_PID_Parameter:");
								OLED_P6x8Str(0*6,1,"Kp: ");	OLED_Show_Float(4,1,Flash_Parameter.Stand_Kp);
								OLED_P6x8Str(0*6,2,"Ki: ");	OLED_Show_Float(4,2,0);
								OLED_P6x8Str(0*6,3,"Kd: ");	OLED_Show_Float(4,3,Flash_Parameter.Stand_Kd);
								OLED_Adjust = &Null;
								break;	
		case 1	:		OLED_P6x8Str(0,0,"Stand_PID_Parameter:");
								OLED_P6x8Str(0*6,1,"Kp: ");	OLED_Show_Float(4,1,Flash_Parameter.Stand_Kp);
								OLED_Adjust = &Flash_Parameter.Stand_Kp;
								break;
		case 2	:		OLED_P6x8Str(0,0,"Stand_PID_Parameter:");
								OLED_P6x8Str(0*6,3,"Kd: ");	OLED_Show_Float(4,3,Flash_Parameter.Stand_Kd);
								OLED_Adjust = &Flash_Parameter.Stand_Kd;
								break;
		
		case 3	:		OLED_P6x8Str(0,0,"Speed_PID_Parameter:");
								OLED_P6x8Str(0*6,1,"Kp: ");	OLED_Show_Float(4,1,Flash_Parameter.Speed_Kp);
								OLED_P6x8Str(0*6,2,"Ki: ");	OLED_Show_Float(4,2,Flash_Parameter.Speed_Ki);
								OLED_P6x8Str(0*6,3,"Kd: ");	OLED_Show_Float(4,3,Flash_Parameter.Speed_Kd);
								OLED_Adjust = &Null;
								break;
		case 4	:		OLED_P6x8Str(0,0,"Speed_PID_Parameter:");
								OLED_P6x8Str(0*6,1,"Kp: ");	OLED_Show_Float(4,1,Flash_Parameter.Speed_Kp);
								OLED_Adjust = &Flash_Parameter.Speed_Kp;
								break;
		case 5	:		OLED_P6x8Str(0,0,"Speed_PID_Parameter:");
								OLED_P6x8Str(0*6,2,"Ki: ");	OLED_Show_Float(4,2,Flash_Parameter.Speed_Ki);
								OLED_Adjust = &Flash_Parameter.Speed_Ki;
								break;						
		case 6	:		OLED_P6x8Str(0,0,"Speed_PID_Parameter:");
								OLED_P6x8Str(0*6,3,"Kd: ");	OLED_Show_Float(4,3,Flash_Parameter.Speed_Kd);
								OLED_Adjust = &Flash_Parameter.Speed_Kd;
								break;
	}
	
}

