#include "IIC.h"
#include "IMU.h"
#include "uart.h"
#include "PID.h"
#include "UART_DMA.h"
#include "ftm.h"

//#define ARM_MATH_CM4
//#include "arm_math.h"
#include "math.h"

int32_t Gyro_Filter[10] = {0};			//滑动平均滤波缓存数组
int16_t Gx_Offset=0,Gy_Offset=0,Gz_Offset=0;		//陀螺仪的零偏
int16_t Gyro_X = 0;
int16_t Gyro_Y = 0;
int16_t Gyro_Z = 0;
int16_t  Acc_X = 0;
int16_t  Acc_Y = 0;
int16_t  Acc_Z = 0;
int16_t  Acc_X_Last = 0;
int16_t  Acc_Y_Last = 0;
int16_t  Acc_Z_Last = 0;
float Gyro_X_Filtered = 0.0;
float Gyro_Y_Filtered = 0.0;
float Gyro_Z_Filtered = 0.0;
float  Acc_X_Filtered = 0.0;
float  Acc_Y_Filtered = 0.0;
float  Acc_Z_Filtered = 0.0;

float Ang_Acc = 0.0;			//根据加速度算出的角度
float Ang_Gyro = 0.0;			//根据陀螺仪算出的角度
float Ang = 0.0;					//互补滤波后的角度
float Gyro_v = 0.0;				//陀螺仪Y轴角速度


/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
	0.02
	6.0000
*/

#define KALMAN_Q        0.02
#define KALMAN_R        6.0000
/*           卡尔曼对三个轴加速度进行滤波处理           */
static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }

static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }
/******** IIC 单字节写 *************************/
void IIC_Single_Write(uint8_t IIC_Address,uint8_t REG_Address,uint8_t REG_data)
{
    IIC_Start();                    //起始信号
    IIC_Send_Byte(IIC_Address);     //发送设备地址+写信号
    IIC_Send_Byte(REG_Address);     //内部寄存器地址， 
    IIC_Send_Byte(REG_data);        //内部寄存器数据， 
    IIC_Stop();                     //发送停止信号
}

/******** IIC 单字节读取内部寄存器 *************/
uint8_t IIC_Single_Read(uint8_t IIC_Address,uint8_t REG_Address)
{  
    uint8_t REG_data;
    IIC_Start();                    //起始信号
    IIC_Send_Byte(IIC_Address);     //发送设备地址+写信号
    IIC_Send_Byte(REG_Address);     //发送存储单元地址，从0开始  
	  
    IIC_Start();                     //起始信号
    IIC_Send_Byte(IIC_Address+1);    //发送设备地址+读信号
    REG_data=IIC_Read_Byte(0);       //读出寄存器数据
    IIC_Stop();                      //停止信号
    return REG_data; 
}
 
/********读 MMA8452 数据*************************/ 
void Read_MMA8452(int16_t *AX,int16_t *AY,int16_t *AZ)
{   
    uint8_t i;
	  uint8_t Xdata[6];                            //数据临时缓存
		
    IIC_Start();                             //起始信号
    IIC_Send_Byte(MMA8452_SlaveAddress);     //发送设备地址+写信号
    IIC_Send_Byte(0x01);                     //发送存储单元地址， 
    IIC_Start();                             //起始信号
    IIC_Send_Byte(MMA8452_SlaveAddress+1);   //发送设备地址+读信号
	
    for (i=0; i<6; i++)                      //连续读取6个地址数据
    {                                             
        if (i < 5)  Xdata[i] = IIC_Read_Byte(1);              //回应ACK    0,1,2,3,4
        else        Xdata[i] = IIC_Read_Byte(0);              //最后一个数据需要回NOACK  5
    }
    IIC_Stop();                            //停止信号
	 
    *AX = (Xdata[0] << 8) | (Xdata[1] );
	  if(Xdata[0]>0x7F) 
		{
			*AX = ~*AX +1;
			*AX *= -1;
		}
		
    *AY = (Xdata[2] << 8) | (Xdata[3] );   
	  if(Xdata[2]>0x7F) 
		{
			*AY = ~*AY +1;
			*AY *= -1;
		}			
		
    *AZ = (Xdata[4] << 8) | (Xdata[5] );  
	  if(Xdata[4]>0x7F) 
		{
			*AZ = ~*AZ +1;
			*AZ *= -1;
		}
}

/********读 L3G4200D 数据*************************/ 
void Read_L3G4200D(int16_t *GX,int16_t *GY,int16_t *GZ)
{   
	uint8_t xMSB,xLSB,yMSB,yLSB,zMSB,zLSB;
 
//	if( ((IIC_Single_Read(L3G4200D_SlaveAddress,L3G4200D_STATUS_REG) & 0x08)>>3) == 1) //陀螺新数据到来
//	{
		xMSB = IIC_Single_Read(L3G4200D_SlaveAddress, 0x29);
		xLSB = IIC_Single_Read(L3G4200D_SlaveAddress, 0x28);
		*GX =  ((xMSB << 8) | xLSB) ;

		yMSB = IIC_Single_Read(L3G4200D_SlaveAddress, 0x2B);
		yLSB = IIC_Single_Read(L3G4200D_SlaveAddress, 0x2A);
		*GY =  ((yMSB << 8) | yLSB) ;

		zMSB = IIC_Single_Read(L3G4200D_SlaveAddress, 0x2D);
		zLSB = IIC_Single_Read(L3G4200D_SlaveAddress, 0x2C);
		*GZ =  ((zMSB << 8) | zLSB) ;	
//	}
}

/**********************初始化 MMA8452 *******************/
uint8_t Init_MMA8452(void)
{
	uint8_t i,ret;

	if(IIC_Single_Read(MMA8452_SlaveAddress,MMA8452_WHO_AM_I) != 0x2A) return 1;   //检测不到加计，返回1
	
	IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_CTRL_REG2 ,0x00);                // 重启芯片
	
	i = 100;
	do 
	{
		ret = IIC_Single_Read(MMA8452_SlaveAddress,MMA8452_CTRL_REG2);
		i--;
	}while (i && (ret & (0x01 << 6)));

	if(i == 0)	 return 1;                          //重启不成功，退出初始化

  IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_CTRL_REG1 ,0x00);   // 进入stanbdy模式	12位数字输出	
	IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_CTRL_REG3 ,0x02);   // 中断脚推挽输出，中断输出高有效，不启动特殊检测模块 
	IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_SYSMOD ,0x01);      // 设置为正常工作模式
	IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_XYZ_DATA_CFG,0x01); // 设置感应量程 ±4g ,关闭高通滤波器 
																																		// 2g： 1024/g	 4g： 512/g		8g： 256/g
  IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_CTRL_REG1 ,0x0D);   // 设置输出速率，400hz，低噪模式，输出模式，激活器件  00000101
 
	return 0;
}

/**********************初始化 L3G4200D **********************/
uint8_t Init_L3G4200D(void)
{
  if(IIC_Single_Read(L3G4200D_SlaveAddress,L3G4200D_WHO_AM_I) != 0xD3) return 1;   //检测不到陀螺，返回1
	IIC_Single_Write(L3G4200D_SlaveAddress, L3G4200D_CTRL_REG1, 0x8F); //400hz z轴还有尖峰7f 200hz会都不见 开启芯片，启动X，Y，Z轴，800Hz输出率  带宽50(cut-off)ef
  IIC_Single_Write(L3G4200D_SlaveAddress, L3G4200D_CTRL_REG2, 0x00); //默认高通滤波设置
  IIC_Single_Write(L3G4200D_SlaveAddress, L3G4200D_CTRL_REG3, 0x00); //关闭所有输出中断 
  IIC_Single_Write(L3G4200D_SlaveAddress, L3G4200D_CTRL_REG4, 0x30); //2000dps    70 mdps/digit
  IIC_Single_Write(L3G4200D_SlaveAddress, L3G4200D_CTRL_REG5, 0x02); //开低通滤波、FIFO	

	return 0;
}

/*********************初始化 L3G4200D 零偏 ******************/
void L3G4200D_InitGyro_Offset(void)
{
	unsigned char i;
	int16_t temp[6] = {0};
	int32_t	tempgx=0,tempgy=0,tempgz=0;

	Gx_Offset=0;
	Gy_Offset=0;
	Gz_Offset=0;
	for(i=0;i<50;i++)
	{
		DelayUs(2500);
		Read_L3G4200D(&temp[0],&temp[1],&temp[2]);
		Read_MMA8452(&temp[3],&temp[4],&temp[5]);
	}
 	for(i=0;i<100;i++)
	{
		DelayUs(2500);
		Read_L3G4200D(&temp[0],&temp[1],&temp[2]);
		Read_MMA8452(&temp[3],&temp[4],&temp[5]);
		tempgx+= temp[0];
		tempgy+= temp[1];
		tempgz+= temp[2];
	}

	Gx_Offset=tempgx/100;
	Gy_Offset=tempgy/100;
	Gz_Offset=tempgz/100;

}

/*********************初始化姿态模块 **********************/
void IMU_Init(void)
{
	IIC_Init();
	DelayMs(10);
	while( Init_MMA8452() )		{	DelayMs(10); }	//初始化MMA8452
	while( Init_L3G4200D() )	{	DelayMs(10); }	//初始化L3G4200D
	DelayMs(10);
	L3G4200D_InitGyro_Offset();
}

/****************陀螺仪传感器值的滑动平均滤波、加速度计传感器值的卡尔曼滤波*******************/
void IMU_Filter(int16_t * GY, int16_t * AX, int16_t * AZ)
{
	static int num = 0;  
	int i=0;
	float sum=0;
	
	//滑动平均滤波
	Gyro_Filter[num] = * GY;
	for(i=0;i<10;i++)
		 sum += Gyro_Filter[i];
	Gyro_Y_Filtered = ( sum / 10.0 ) - Gy_Offset;
	num = (num + 1) % 10;
	
	Acc_X_Filtered = KalmanFilter_x( *AX, KALMAN_Q, KALMAN_R );  // ACC X轴卡尔曼滤波   
	Acc_Z_Filtered = KalmanFilter_z( *AZ, KALMAN_Q, KALMAN_R );  // ACC Z轴卡尔曼滤波 

}

#define ANG_TO_RAD 57.295779513	//	180/3.141592
/*********************读传感器、滤波、算出角度、PID_Stand*********************/
void IMU_Update(void)
{
	float temp[6] = {0};

	if( ((IIC_Single_Read(L3G4200D_SlaveAddress,L3G4200D_STATUS_REG) & 0x08)>>3) == 1) //陀螺新数据到来
		Read_L3G4200D(&Gyro_X,&Gyro_Y,&Gyro_Z);  //读数据 
	if( ((IIC_Single_Read(MMA8452_SlaveAddress,MMA8452_STATUS) & 0x08)>>3) == 1) //加速度新数据到来
		Read_MMA8452(&Acc_X,&Acc_Y,&Acc_Z);  //读数据 	
	
	if(Acc_Y == Acc_X)				//软件滤掉Acc大的尖峰
		Acc_Y = Acc_Y_Last;
	if(Acc_Z == Acc_Y)
		Acc_Z = Acc_Z_Last;
	Acc_X_Last = Acc_X;
	Acc_Y_Last = Acc_Y;
	Acc_Z_Last = Acc_Z;
	
	IMU_Filter(&Gyro_Y,&Acc_X,&Acc_Z);
	
	Ang_Acc = atan2(Acc_Z_Filtered,Acc_X_Filtered) * ANG_TO_RAD;    //弧度转成角度 求z/x的反正切，本来应右移四位根据不同量程乘上转换系数,这里刚好分母分子约去
	Gyro_v = Gyro_Y_Filtered * 0.07;	//	70mdps/digit	算得角速度，直立PID用到
	Ang = 0.98 *  (Ang + Gyro_v * 0.005) 	+ 0.02 * Ang_Acc; // 采集周期5ms	 	互补滤波
	
	
//	temp[0] = Ang - 0.02 * Ang_Acc;		//陀螺仪积分
//	temp[1] = Ang;   									//互补的角度
//	UART_DMA_Array_Width_Six(temp);
	
}
