#include "Drv_Yaw.h"
#include "Ano_Pid.h"
#include "Drv_mv1.h"
#include "Drv_mv.h"
#include "ANO_Math.h"
#include "LX_FC_Fun.h"
#include "Ano_FlightCtrl.h"

//YAW�����ǶȻ����Ʋ���
_PID_arg_st my_yaw_arg21[2] ; 

//YAW�����ǶȻ���������
_PID_val_st my_yaw_val_2[2] ;
_PID_val_st my_yaw_val_1[2] ;

_loc_ctrl_st my_yaw_fix;

extern struct MYMODE my_flight;
extern struct MYMODE my_flight_mv1;
extern float Yaw_Dps_Out;

void User_my_yaw_PID_Init(void)
{
 		my_yaw_arg21[X].kp = 0.75f;  
		my_yaw_arg21[X].ki = 0.0f;
		my_yaw_arg21[X].kd_ex = 0.00f ;
		my_yaw_arg21[X].kd_fb = 0.15f;
		my_yaw_arg21[X].k_ff  =  0.00f;
	  my_yaw_arg21[Y] = my_yaw_arg21[X];
}

extern struct _line_check_ line;
extern struct _dot_check_ dot_mv1;
extern struct _line_check_ line_mv1;
extern struct _dot_check_ dot;
float Yaw_Dot_out  =0;
float Yaw_Line_out =0;
extern  u8 flag_trackingoff;

u8 yaw_free;

void User_my_yaw_2level(float dT)
{
	
//    float out = 0.0f;
	
//	  my_yaw_fix.exp[X]=0.0f;
	
//	if(my_flight_mv1.mode == Follow_Dot_Mode)    //�Ƹ�
//	{
//		if(dot_mv1.ok)
//		{
//			 my_yaw_fix.fb[X] = dot_mv1.x;
//		}
//		else
//		{
//			 my_yaw_fix.fb[X] = 0.0f;
//		}
//					
//		PID_calculate( dT*1e-3f,            //���ڣ���λ���룩
//								0 ,				//ǰ��ֵ
//								my_yaw_fix.exp[X] ,				//����ֵ���趨ֵ��
//								my_yaw_fix.fb[X] ,			//����ֵ����
//								&my_yaw_arg21[X], //PID�����ṹ��
//								&my_yaw_val_2[X],	//PID���ݽṹ��
//								50,//��������޷�
//								10	*flag_trackingoff//integration limit�������޷�
//							)	;
//					my_yaw_val_2[X].out = my_yaw_val_2[X].out *1;
//					if(my_yaw_val_2[X].out>30.0f)
//					my_yaw_val_2[X].out=30.0f;
//					if(my_yaw_val_2[X].out<-30.0f)
//					my_yaw_val_2[X].out=-30.0f;
//	 }	
	
//	 if(my_flight_mv1.mode == Follow_Line_Mode)   //Ѳ������YAW
//	 {
//				if(line_mv1.angle_ok)
//				{
//				  my_yaw_fix.fb[X] = line_mv1.angle_y;
//					my_yaw_fix.fb[Y] = line_mv1.angle_x;
//				}
//				else 
//				{
//				   my_yaw_fix.fb[X]= 0.0f;
//				}
//		 			PID_calculate( dT*1e-3f,            //���ڣ���λ���룩
//											0 ,				//ǰ��ֵ
//											my_yaw_fix.exp[X] ,				//����ֵ���趨ֵ��
//											my_yaw_fix.fb[X] ,			//����ֵ����
//											&my_yaw_arg21[X], //PID�����ṹ��
//											&my_yaw_val_1[X],	//PID���ݽṹ��
//											50,//��������޷�
//												10 * flag_trackingoff			//integration limit�������޷�
//										);
//				  my_yaw_val_1[X].out = my_yaw_val_1[X].out *1;
//					if(my_yaw_val_1[X].out >15.0f)
//					my_yaw_val_1[X].out =15.0f;
//					if(my_yaw_val_1[X].out <-15.0f)
//					my_yaw_val_1[X].out =-15.0f;
//	 }
//	 
//		switch(my_flight_mv1.mode)   
//		{
//		  case Follow_Dot_Mode: Yaw_Dps_Out = -my_yaw_val_2[X].out;   break;
//			case Follow_Line_Mode: Yaw_Dps_Out = -my_yaw_val_1[X].out; break;
//			case Follow_RGDot_Mode: Yaw_Dps_Out = -my_yaw_val_1[X].out;    break;
//			case Follow_Null: Yaw_Dps_Out = 0; break;
//		  default: Yaw_Dps_Out =0;  break;	
//		}					
}

