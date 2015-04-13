#ifndef __OLED_H__  
#define __OLED_H__  

#include "common.h"  
#include "FlashOS.H"

#include "KEY.h"
#include "PID.h"

//IO��������
#define OLED_IN()  {PTB->PDDR &= ~(1 << 1);}
#define OLED_OUT() {PTB->PDDR |= (1 << 1);}

//IO��������	 
#define OLED_SCL    PBout(0) //SCL
#define OLED_SDA    PBout(1) //SDA	 
#define READ_OLED_SDA   PBin(1)  //����SDA

//OLED����
#define XLevelL     0x00  
#define XLevelH     0x10  
#define XLevel      ((XLevelH&0x0F)*16+XLevelL)  
#define Max_Column  128  
#define Max_Row     64  
#define Brightness  0xCF   
#define X_WIDTH 128  
#define Y_WIDTH 64


//OLED_IIC��ʼ��
void OLED_IIC_Init(void);        	//��ʼ��OLED_IIC	 
void OLED_IIC_Start(void);				//����OLED_IIC��ʼ�ź�
void OLED_IIC_Stop(void);	  			//����OLED_IICֹͣ�ź�
void OLED_IIC_Send_Byte(uint8_t txd);			//OLED_IIC����һ���ֽ�
uint8_t OLED_IIC_Read_Byte(uint8_t ack);	//OLED_IIC��ȡһ���ֽ�
uint8_t OLED_IIC_Wait_Ack(void); 					//OLED_IIC�ȴ�ACK�ź�
void OLED_IIC_Ack(void);					//OLED_IIC����ACK�ź�
void OLED_IIC_NAck(void);					//OLED_IIC������ACK�ź�
void OLED_Delay(uint32_t time);	     
 

 
void OLED_WRCMD(uint8_t com);  
void OLED_WRDATA(uint8_t dat);  
void OLED_Set_Pos(uint8_t x, uint8_t y);  //OLED�������� 
void OLED_CLS(void);		//OLED��λ
void OLED_Init(void);		//OLED��ʼ��    
void OLED_P6x8Char(uint8_t x,uint8_t y,uint8_t dat);	//��ʾ6*8һ��ASCII�ַ�	��ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_P8x16Str(uint8_t x,uint8_t y,char ch[]);		//��ʾ8*16һ���׼ASCII�ַ���	 ��ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_PutPixel(uint8_t x,uint8_t y);	//����
void OLED_P16x16Ch(uint8_t x, uint8_t y, uint8_t N);	//������������ʾ16*16����  ��ʾ�����꣨x,y����yΪҳ��Χ0��7
void Draw_BMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,uint8_t BMP[]); //������������ʾ��ʾBMPͼƬ128��64��ʼ������(x,y),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7

void OLED_Init(void); 
void OLED_Fill(uint8_t bmp_dat);  //bmp_dat=0x00ȫ����,bmp_dat=0xffȫ����
void OLED_P6x8Str(uint8_t x, uint8_t y,char ch[]);		//��ʾ6*8һ���׼ASCII�ַ���	��ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_Show_Data(uint8_t x,uint8_t y,int32_t var);
void OLED_Show_Float(uint8_t x,uint8_t y,float var);	//��ʾһλС���İ�λ������������ 
void OLED_UI(void);
/************************FLASH*******************************/
/* chip's Flash size and sector size can be found in RM */
#define FLASH_SIZE      0x80000
#define SECTER_SIZE     0x000800

/* get  residue flash size */
#define TEST_ADDR_BEIGN  (uint32_t)(FLASH_SIZE - SECTER_SIZE)

int MKP512FlashInit(void);

#endif  

