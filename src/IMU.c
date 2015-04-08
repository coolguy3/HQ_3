#include "IIC.h"
#include "IMU.h"
#include "uart.h"
#include "PID.h"
#include "UART_DMA.h"
#include "ftm.h"

//#define ARM_MATH_CM4
//#include "arm_math.h"
#include "math.h"

int32_t Gyro_Filter[10] = {0};			//����ƽ���˲���������
int16_t Gx_Offset=0,Gy_Offset=0,Gz_Offset=0;		//�����ǵ���ƫ
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

float Ang_Acc = 0.0;			//���ݼ��ٶ�����ĽǶ�
float Ang_Gyro = 0.0;			//��������������ĽǶ�
float Ang = 0.0;					//�����˲���ĽǶ�
float Gyro_v = 0.0;				//������Y����ٶ�


/*	
	Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
	R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��	
	0.02
	6.0000
*/

#define KALMAN_Q        0.02
#define KALMAN_R        6.0000
/*           ����������������ٶȽ����˲�����           */
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
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
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
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;                
 }
/******** IIC ���ֽ�д *************************/
void IIC_Single_Write(uint8_t IIC_Address,uint8_t REG_Address,uint8_t REG_data)
{
    IIC_Start();                    //��ʼ�ź�
    IIC_Send_Byte(IIC_Address);     //�����豸��ַ+д�ź�
    IIC_Send_Byte(REG_Address);     //�ڲ��Ĵ�����ַ�� 
    IIC_Send_Byte(REG_data);        //�ڲ��Ĵ������ݣ� 
    IIC_Stop();                     //����ֹͣ�ź�
}

/******** IIC ���ֽڶ�ȡ�ڲ��Ĵ��� *************/
uint8_t IIC_Single_Read(uint8_t IIC_Address,uint8_t REG_Address)
{  
    uint8_t REG_data;
    IIC_Start();                    //��ʼ�ź�
    IIC_Send_Byte(IIC_Address);     //�����豸��ַ+д�ź�
    IIC_Send_Byte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ  
	  
    IIC_Start();                     //��ʼ�ź�
    IIC_Send_Byte(IIC_Address+1);    //�����豸��ַ+���ź�
    REG_data=IIC_Read_Byte(0);       //�����Ĵ�������
    IIC_Stop();                      //ֹͣ�ź�
    return REG_data; 
}
 
/********�� MMA8452 ����*************************/ 
void Read_MMA8452(int16_t *AX,int16_t *AY,int16_t *AZ)
{   
    uint8_t i;
	  uint8_t Xdata[6];                            //������ʱ����
		
    IIC_Start();                             //��ʼ�ź�
    IIC_Send_Byte(MMA8452_SlaveAddress);     //�����豸��ַ+д�ź�
    IIC_Send_Byte(0x01);                     //���ʹ洢��Ԫ��ַ�� 
    IIC_Start();                             //��ʼ�ź�
    IIC_Send_Byte(MMA8452_SlaveAddress+1);   //�����豸��ַ+���ź�
	
    for (i=0; i<6; i++)                      //������ȡ6����ַ����
    {                                             
        if (i < 5)  Xdata[i] = IIC_Read_Byte(1);              //��ӦACK    0,1,2,3,4
        else        Xdata[i] = IIC_Read_Byte(0);              //���һ��������Ҫ��NOACK  5
    }
    IIC_Stop();                            //ֹͣ�ź�
	 
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

/********�� L3G4200D ����*************************/ 
void Read_L3G4200D(int16_t *GX,int16_t *GY,int16_t *GZ)
{   
	uint8_t xMSB,xLSB,yMSB,yLSB,zMSB,zLSB;
 
//	if( ((IIC_Single_Read(L3G4200D_SlaveAddress,L3G4200D_STATUS_REG) & 0x08)>>3) == 1) //���������ݵ���
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

/**********************��ʼ�� MMA8452 *******************/
uint8_t Init_MMA8452(void)
{
	uint8_t i,ret;

	if(IIC_Single_Read(MMA8452_SlaveAddress,MMA8452_WHO_AM_I) != 0x2A) return 1;   //��ⲻ���Ӽƣ�����1
	
	IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_CTRL_REG2 ,0x00);                // ����оƬ
	
	i = 100;
	do 
	{
		ret = IIC_Single_Read(MMA8452_SlaveAddress,MMA8452_CTRL_REG2);
		i--;
	}while (i && (ret & (0x01 << 6)));

	if(i == 0)	 return 1;                          //�������ɹ����˳���ʼ��

  IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_CTRL_REG1 ,0x00);   // ����stanbdyģʽ	12λ�������	
	IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_CTRL_REG3 ,0x02);   // �жϽ�����������ж��������Ч��������������ģ�� 
	IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_SYSMOD ,0x01);      // ����Ϊ��������ģʽ
	IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_XYZ_DATA_CFG,0x01); // ���ø�Ӧ���� ��4g ,�رո�ͨ�˲��� 
																																		// 2g�� 1024/g	 4g�� 512/g		8g�� 256/g
  IIC_Single_Write(MMA8452_SlaveAddress,MMA8452_CTRL_REG1 ,0x0D);   // ����������ʣ�400hz������ģʽ�����ģʽ����������  00000101
 
	return 0;
}

/**********************��ʼ�� L3G4200D **********************/
uint8_t Init_L3G4200D(void)
{
  if(IIC_Single_Read(L3G4200D_SlaveAddress,L3G4200D_WHO_AM_I) != 0xD3) return 1;   //��ⲻ�����ݣ�����1
	IIC_Single_Write(L3G4200D_SlaveAddress, L3G4200D_CTRL_REG1, 0x8F); //400hz z�ỹ�м��7f 200hz�ᶼ���� ����оƬ������X��Y��Z�ᣬ800Hz�����  ����50(cut-off)ef
  IIC_Single_Write(L3G4200D_SlaveAddress, L3G4200D_CTRL_REG2, 0x00); //Ĭ�ϸ�ͨ�˲�����
  IIC_Single_Write(L3G4200D_SlaveAddress, L3G4200D_CTRL_REG3, 0x00); //�ر���������ж� 
  IIC_Single_Write(L3G4200D_SlaveAddress, L3G4200D_CTRL_REG4, 0x30); //2000dps    70 mdps/digit
  IIC_Single_Write(L3G4200D_SlaveAddress, L3G4200D_CTRL_REG5, 0x02); //����ͨ�˲���FIFO	

	return 0;
}

/*********************��ʼ�� L3G4200D ��ƫ ******************/
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

/*********************��ʼ����̬ģ�� **********************/
void IMU_Init(void)
{
	IIC_Init();
	DelayMs(10);
	while( Init_MMA8452() )		{	DelayMs(10); }	//��ʼ��MMA8452
	while( Init_L3G4200D() )	{	DelayMs(10); }	//��ʼ��L3G4200D
	DelayMs(10);
	L3G4200D_InitGyro_Offset();
}

/****************�����Ǵ�����ֵ�Ļ���ƽ���˲������ٶȼƴ�����ֵ�Ŀ������˲�*******************/
void IMU_Filter(int16_t * GY, int16_t * AX, int16_t * AZ)
{
	static int num = 0;  
	int i=0;
	float sum=0;
	
	//����ƽ���˲�
	Gyro_Filter[num] = * GY;
	for(i=0;i<10;i++)
		 sum += Gyro_Filter[i];
	Gyro_Y_Filtered = ( sum / 10.0 ) - Gy_Offset;
	num = (num + 1) % 10;
	
	Acc_X_Filtered = KalmanFilter_x( *AX, KALMAN_Q, KALMAN_R );  // ACC X�Ῠ�����˲�   
	Acc_Z_Filtered = KalmanFilter_z( *AZ, KALMAN_Q, KALMAN_R );  // ACC Z�Ῠ�����˲� 

}

#define ANG_TO_RAD 57.295779513	//	180/3.141592
/*********************�����������˲�������Ƕȡ�PID_Stand*********************/
void IMU_Update(void)
{
	float temp[6] = {0};

	if( ((IIC_Single_Read(L3G4200D_SlaveAddress,L3G4200D_STATUS_REG) & 0x08)>>3) == 1) //���������ݵ���
		Read_L3G4200D(&Gyro_X,&Gyro_Y,&Gyro_Z);  //������ 
	if( ((IIC_Single_Read(MMA8452_SlaveAddress,MMA8452_STATUS) & 0x08)>>3) == 1) //���ٶ������ݵ���
		Read_MMA8452(&Acc_X,&Acc_Y,&Acc_Z);  //������ 	
	
	if(Acc_Y == Acc_X)				//����˵�Acc��ļ��
		Acc_Y = Acc_Y_Last;
	if(Acc_Z == Acc_Y)
		Acc_Z = Acc_Z_Last;
	Acc_X_Last = Acc_X;
	Acc_Y_Last = Acc_Y;
	Acc_Z_Last = Acc_Z;
	
	IMU_Filter(&Gyro_Y,&Acc_X,&Acc_Z);
	
	Ang_Acc = atan2(Acc_Z_Filtered,Acc_X_Filtered) * ANG_TO_RAD;    //����ת�ɽǶ� ��z/x�ķ����У�����Ӧ������λ���ݲ�ͬ���̳���ת��ϵ��,����պ÷�ĸ����Լȥ
	Gyro_v = Gyro_Y_Filtered * 0.07;	//	70mdps/digit	��ý��ٶȣ�ֱ��PID�õ�
	Ang = 0.98 *  (Ang + Gyro_v * 0.005) 	+ 0.02 * Ang_Acc; // �ɼ�����5ms	 	�����˲�
	
	
//	temp[0] = Ang - 0.02 * Ang_Acc;		//�����ǻ���
//	temp[1] = Ang;   									//�����ĽǶ�
//	UART_DMA_Array_Width_Six(temp);
	
}
