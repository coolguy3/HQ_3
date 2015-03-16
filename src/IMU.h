#ifndef __IMU_H
#define __IMU_H
#include "common.h"


#define	MMA8452_SlaveAddress    0x38	  //SA0 = 0   ---> b0011100 X   MMA8452  通讯地址
#define	L3G4200D_SlaveAddress   0xD2	  //SDO = 1   ---> b1101001 X   L3G4200D 通讯地址


//MMA8452 寄存器地址
#define MMA8452_STATUS            0x00     //R,   数据状态寄存器，
#define MMA8452_OUT_X_MSB         0x01     //R,   X轴数据输出高端
#define MMA8452_OUT_X_LSB         0x02     //R,   X轴数据输出低端
#define MMA8452_OUT_Y_MSB         0x03     //R,   Y轴数据输出高端
#define MMA8452_OUT_Y_LSB     	  0x04     //R,   Y轴数据输出低端
#define MMA8452_OUT_Z_MSB     	  0x05     //R,   Z轴数据输出高端
#define MMA8452_OUT_Z_LSB     	  0x06     //R,   Z轴数据输出低端
#define MMA8452_F_SETUP     		  0x09     //R/W, 设置FIFO 
#define MMA8452_TRIG_CFG        	0x0A     //R/W, 设置内部功能检测模块
#define MMA8452_SYSMOD            0x0B     //R/W, 设置工作模式  
#define MMA8452_INT_SOURCE        0x0C     //R,   中断状态寄存器
#define MMA8452_WHO_AM_I          0x0D     //R,   识别寄存器
#define MMA8452_XYZ_DATA_CFG      0x0E     //R/W, 设置量程及使能高通滤波
#define MMA8452_HP_FILTER_CUTOFF  0x0F     //R/W, 设置高通滤波器
#define MMA8452_PL_STATUS         0x10    
#define MMA8452_PL_CFG            0x11    
#define MMA8452_PL_COUNT          0x12    
#define MMA8452_PL_BF_ZCOMP       0x13    
#define MMA8452_PL_THS_REG        0x14    
#define MMA8452_FF_MT_CFG         0x15    
#define MMA8452_FF_MT_SRC      	  0x16    
#define MMA8452_FF_MT_THS      		0x17    
#define MMA8452_FF_MT_COUNT       0x18    
#define MMA8452_TRANSIENT_CFG     0x1D    
#define MMA8452_TRANSIENT_SCR     0x1E    
#define MMA8452_TRANSIENT_THS     0x1F    
#define MMA8452_TRANSIENT_CUNT    0x20    
#define MMA8452_PULSE_CFG         0x21    
#define MMA8452_PULSE_SRC         0x22    
#define MMA8452_PULSE_THSX        0x23    
#define MMA8452_PULSE_THSY        0x24    
#define MMA8452_PULSE_THSZ    	  0x25    
#define MMA8452_PULSE_TMLT    	  0x26    
#define MMA8452_PULSE_LTCY    	  0x27    
#define MMA8452_PULSE_WIND    	  0x28    
#define MMA8452_ASLP_COUNT    	  0x29    
#define MMA8452_CTRL_REG1     	  0x2A   //R/W,  设置状态   
#define MMA8452_CTRL_REG2      	  0x2B    
#define MMA8452_CTRL_REG3      	  0x2C    
#define MMA8452_CTRL_REG4      	  0x2D    
#define MMA8452_CTRL_REG5      	  0x2E    
#define MMA8452_OFF_X       			0x2F    
#define MMA8452_OFF_Y       			0x30
#define MMA8452_OFF_Z       			0x31    


//L3G4200D 寄存器地址
#define L3G4200D_WHO_AM_I 			  0x0F
#define L3G4200D_CTRL_REG1 				0x20
#define L3G4200D_CTRL_REG2 				0x21
#define L3G4200D_CTRL_REG3 				0x22
#define L3G4200D_CTRL_REG4 				0x23
#define L3G4200D_CTRL_REG5 				0x24
#define L3G4200D_REFERENCE 				0x25
#define L3G4200D_OUT_TEMP 				0x26
#define L3G4200D_STATUS_REG 			0x27
#define L3G4200D_OUT_X_L 					0x28
#define L3G4200D_OUT_X_H 					0x29
#define L3G4200D_OUT_Y_L 					0x2A
#define L3G4200D_OUT_Y_H 					0x2B
#define L3G4200D_OUT_Z_L 					0x2C
#define L3G4200D_OUT_Z_H 					0x2D
#define L3G4200D_FIFO_CTRL_REG 		0x2E
#define L3G4200D_FIFO_SRC_REG 		0x2F
#define L3G4200D_INT1_CFG 				0x30
#define L3G4200D_INT1_SRC 				0x31
#define L3G4200D_INT1_TSH_XH 			0x32
#define L3G4200D_INT1_TSH_XL 			0x33
#define L3G4200D_INT1_TSH_YH 			0x34
#define L3G4200D_INT1_TSH_YL 			0x35
#define L3G4200D_INT1_TSH_ZH 			0x36
#define L3G4200D_INT1_TSH_ZL 			0x37
#define L3G4200D_INT1_DURATION 		0x38
 

void Read_MMA8452(int16_t *AX,int16_t *AY,int16_t *AZ);    //读MMA8452数据
void Read_L3G4200D(int16_t *GX,int16_t *GY,int16_t *GZ);   //读L3G4200D数据
uint8_t Init_L3G4200D(void);     //初始化L3G4200D
uint8_t Init_MMA8452(void);      //初始化MMA8452
void L3G4200D_InitGyro_Offset(void);
uint8_t IIC_Single_Read(uint8_t IIC_Address,uint8_t REG_Address);                 //发起IIC单字节读取协议
void IIC_Single_Write(uint8_t IIC_Address,uint8_t REG_Address,uint8_t REG_data);  //发起IIC单字节写协议
void IMU_Init(void);
void IMU_Update(void);
void IMU_Filter(int16_t * GX, int16_t * GY, int16_t * GZ, int16_t * AX, int16_t * AY, int16_t * AZ);
void IMU_Report(float * GX, float * GY, float * GZ, float * AX, float * AY, float * AZ);
	
#endif




