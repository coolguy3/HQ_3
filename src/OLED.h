#ifndef __OLED_H__  
#define __OLED_H__  

#include "common.h"  
#include "FlashOS.H"

#include "KEY.h"
#include "PID.h"

//IO方向设置
#define OLED_IN()  {PTB->PDDR &= ~(1 << 1);}
#define OLED_OUT() {PTB->PDDR |= (1 << 1);}

//IO操作函数	 
#define OLED_SCL    PBout(0) //SCL
#define OLED_SDA    PBout(1) //SDA	 
#define READ_OLED_SDA   PBin(1)  //输入SDA

//OLED配置
#define XLevelL     0x00  
#define XLevelH     0x10  
#define XLevel      ((XLevelH&0x0F)*16+XLevelL)  
#define Max_Column  128  
#define Max_Row     64  
#define Brightness  0xCF   
#define X_WIDTH 128  
#define Y_WIDTH 64


//OLED_IIC初始化
void OLED_IIC_Init(void);        	//初始化OLED_IIC	 
void OLED_IIC_Start(void);				//发送OLED_IIC开始信号
void OLED_IIC_Stop(void);	  			//发送OLED_IIC停止信号
void OLED_IIC_Send_Byte(uint8_t txd);			//OLED_IIC发送一个字节
uint8_t OLED_IIC_Read_Byte(uint8_t ack);	//OLED_IIC读取一个字节
uint8_t OLED_IIC_Wait_Ack(void); 					//OLED_IIC等待ACK信号
void OLED_IIC_Ack(void);					//OLED_IIC发送ACK信号
void OLED_IIC_NAck(void);					//OLED_IIC不发送ACK信号
void OLED_Delay(uint32_t time);	     
 

 
void OLED_WRCMD(uint8_t com);  
void OLED_WRDATA(uint8_t dat);  
void OLED_Set_Pos(uint8_t x, uint8_t y);  //OLED设置坐标 
void OLED_CLS(void);		//OLED复位
void OLED_Init(void);		//OLED初始化    
void OLED_P6x8Char(uint8_t x,uint8_t y,uint8_t dat);	//显示6*8一个ASCII字符	显示的坐标（x,y），y为页范围0～7
void OLED_P8x16Str(uint8_t x,uint8_t y,char ch[]);		//显示8*16一组标准ASCII字符串	 显示的坐标（x,y），y为页范围0～7
void OLED_PutPixel(uint8_t x,uint8_t y);	//画点
void OLED_P16x16Ch(uint8_t x, uint8_t y, uint8_t N);	//功能描述：显示16*16点阵  显示的坐标（x,y），y为页范围0～7
void Draw_BMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,uint8_t BMP[]); //功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7

void OLED_Init(void); 
void OLED_Fill(uint8_t bmp_dat);  //bmp_dat=0x00全屏灭,bmp_dat=0xff全屏亮
void OLED_P6x8Str(uint8_t x, uint8_t y,char ch[]);		//显示6*8一组标准ASCII字符串	显示的坐标（x,y），y为页范围0～7
void OLED_Show_Data(uint8_t x,uint8_t y,int32_t var);
void OLED_Show_Float(uint8_t x,uint8_t y,float var);	//显示一位小数的百位数浮点型数据 
void OLED_UI(void);
/************************FLASH*******************************/
/* chip's Flash size and sector size can be found in RM */
#define FLASH_SIZE      0x80000
#define SECTER_SIZE     0x000800

/* get  residue flash size */
#define TEST_ADDR_BEIGN  (uint32_t)(FLASH_SIZE - SECTER_SIZE)

int MKP512FlashInit(void);

#endif  

