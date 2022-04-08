#include "Ano_Alt.h"
#include "Ano_Pid.h"
#include "Ano_FlightCtrl.h"
#include "Drv_AnoOf.h"

//�߶ȿ��Ʋ���
_PID_arg_st my_alt_arg_1; 

//�߶ȿ�������
_PID_val_st my_alt_val_1;

_loc_ctrl_st my_alt_fix;

float User_Alt_ctrl;
extern u8 flag_trackingoff;
extern float Yaw_Dps_Out;
extern float Vel_X_Out;
extern float Vel_Y_Out;
extern float Vel_Z_Out;

void User_Alt_PID_Init(void)  //�߶�PID
{
 		my_alt_arg_1.kp = 1.0f;  
		my_alt_arg_1.ki = 0.0f;
		my_alt_arg_1.kd_ex = 0.00f ;
		my_alt_arg_1.kd_fb = 0.05f;
		my_alt_arg_1.k_ff  =  0.00f;	 
}

extern _ano_of_st ano_of;  //�����������

void Alt_level(float dT)
{
	  my_alt_fix.exp[Z]= 145;
		
		my_alt_fix.fb[Z] = ano_of.of_alt_cm;
	
			PID_calculate( dT*1e-3f,            //���ڣ���λ���룩
											0 ,				//ǰ��ֵ
											my_alt_fix.exp[Z],				//����ֵ���趨ֵ��
											my_alt_fix.fb[Z] ,			//����ֵ����
											&my_alt_arg_1, //PID�����ṹ��
											&my_alt_val_1,	//PID���ݽṹ��
											100,//��������޷�
												10 *flag_trackingoff			//integration limit�������޷�
										)	;
	
	  my_alt_val_1.out = my_alt_val_1.out *1;
    if(my_alt_val_1.out>15.0f)  my_alt_val_1.out=15.0f;
		if(my_alt_val_1.out<-15.0f)	my_alt_val_1.out=-15.0f;
	  
	if(User_Alt_ctrl==1)
	  Vel_Z_Out = my_alt_val_1.out;	
  else   Vel_Z_Out=0;
}

