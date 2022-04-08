#include "Ano_LocCtrl.h"
#include "Ano_FlightCtrl.h"
#include "Drv_mv.h"
#include "math.h"
#include "Drv_bluetooch.h"
#include "Drv_Uart.h"
#include "Drv_mv1.h"
#include "Drv_Yaw.h"
#include "LX_FC_State.h"
#include "LX_FC_Fun.h"
#include "Drv_beep.h"
#include "Drv_AnoOf.h"
#include "LX_FC_State.h"

#define TRACKING_SPEED 15
#define LOW_SPEED  8
#define S 0 //��ͣ
#define F 1 //ǰ��
#define B 2 //���
#define L 3 //����
#define R 4 //����

extern float Yaw_Dps_Out;
extern float Vel_X_Out;
extern float Vel_Y_Out;
extern float Vel_Z_Out;

//�û�λ�û����Ʋ��� ����
_PID_arg_st len_arg_2[2] ; 

//�û�λ�û���������  ����
_PID_val_st len_val_2[2] ;

//�û�λ�û����Ʋ���
_PID_arg_st loc_arg_2[2] ; 

//�û�λ�û���������
_PID_val_st loc_val_2[2] ;

//ѭ��ƫ����Ʋ���
_PID_arg_st line_arg_2[2] ; 

//ѭ��ƫ���������
_PID_val_st line_val_2[2] ;

_tracking_ctrl_st tracking1;   //Ѳ��ʱ�����̿��� 
_tracking_ctrl_st tracking2;
_tracking_ctrl_st tracking3;   //Ѳ��ʱ�����̿��� 
_tracking_ctrl_st tracking4;
_tracking_ctrl_st tracking5;   //Ѳ��ʱ�����̿��� 
_tracking_ctrl_st tracking6;


void User_line_PID_Init()    //
{
 		line_arg_2[X].kp = 0.20f;  
		line_arg_2[X].ki = 0.00f;
		line_arg_2[X].kd_ex = 0.00f ;
		line_arg_2[X].kd_fb = 0.05f;
		line_arg_2[X].k_ff = 0.00f;
	  line_arg_2[Y] = line_arg_2[X];
}

void User_Loc_PID_Init()   //׷��Ѳ��PID
{
 		loc_arg_2[X].kp = 0.25f;    
		loc_arg_2[X].ki = 0.00f;
		loc_arg_2[X].kd_ex = 0.00f ;
		loc_arg_2[X].kd_fb = 0.07f;
		loc_arg_2[X].k_ff = 0.00f;
	  loc_arg_2[Y] = loc_arg_2[X];
}

void User_Len_PID_Init()  
{
 		len_arg_2[X].kp = 0.35f;  
		len_arg_2[X].ki = 0.00f;
		len_arg_2[X].kd_ex = 0.00f ;
		len_arg_2[X].kd_fb = 0.15f;
		len_arg_2[X].k_ff = 0.00f;
	  len_arg_2[Y] = len_arg_2[X];
}

_loc_ctrl_st ur_loc_c;    //��openmv�ҵ�
_loc_ctrl_st len_loc_c;   //ǰopenmv�Ƹ�
_loc_ctrl_st line_loc_c;  //ǰopenmvѲ��

extern struct MYMODE my_flight;     //��openmv
extern struct MYMODE my_flight_mv1;  //ǰopenmv 
extern struct _line_check_ line;
extern struct _line_check_ line_mv1;
extern struct _dot_check_ dot;
extern struct _dot_check_ dot_mv1;
extern _ano_of_st ano_of; 
extern u8 flag_trackingoff;
extern u8 beep_flag;
extern u8 yaw_free;
extern u8 one_Kland;
extern float my_yaw;
extern u8 one_Kland;
u8 loc_free_x=0;    //Y���������Ƿ���Ч
extern _fc_state_st fc_sta;
extern 	u8 YAW_flag;
extern u8 Dot_flag;
extern u8 Pro_Ctr_step; 
u8 contral_mode=0;
u8 ks_free;
u8 cross_step = 0;
u8 dot_two_mode;

void Loc_Ctrl(u16 dT_ms,struct _MV_ *mv)
{
	MV_Fix(2*1e-2f);						    		//�Ӿ����ݴ���
	MV_Flag_Check(line.flag,dot.flag);  //������ͷ
  //MV1_Flag_Check(line_mv1.flag,dot_mv1.flag);  //�´�����ͷ
	if(my_flight.mode == Follow_Dot_Mode  || my_flight.mode == Follow_RGDot_Mode)   
	{			

		  if(dot_two_mode == 1)
			{
			 ur_loc_c.exp[X] = -20; 
			 ur_loc_c.exp[Y] = 0; 
			}
			
			if(dot_two_mode ==0)
				{
		   ur_loc_c.exp[X] = 0; 
			 ur_loc_c.exp[Y] = 0;
				}
				
				
				if(dot.ok == 1)   //��ͷ����Ϊx��
				{			 				
					ur_loc_c.fb[X] = mv->fix_y;     //�϶��½����
					ur_loc_c.fb[Y] = mv->fix_x;;	
				}
					
				else
				{
					ur_loc_c.fb[X] = 0;      //���㷴����ֵ//�϶��½����
					ur_loc_c.fb[Y] = 0;
				}
				PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
											0 ,				//ǰ��ֵ
											ur_loc_c.exp[X] ,				//����ֵ���趨ֵ��
											ur_loc_c.fb[X] ,			//����ֵ���� 
											&loc_arg_2[X], //PID�����ṹ��
											&loc_val_2[X],	//PID���ݽṹ��
											50,//��������޷�
											10 	*flag_trackingoff		//integration limit�������޷�
											);

				PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
											0 ,				//ǰ��ֵ
											ur_loc_c.exp[Y] ,				//����ֵ���趨ֵ��
											ur_loc_c.fb[Y] ,			//����ֵ���� 
											&loc_arg_2[Y], //PID�����ṹ��
											&loc_val_2[Y],	//PID���ݽṹ��
											50,//��������޷�
											10 	*flag_trackingoff		//integration limit�������޷�
											);	
				
						switch (Pro_Ctr_step)
						{
						  case 1:loc_val_2[X].out = loc_val_2[X].out  *1 +tracking1.x; break;
						  case 2:loc_val_2[X].out = loc_val_2[X].out  *1 +tracking2.x; break;						
						  case 3:loc_val_2[X].out = loc_val_2[X].out  *1 +tracking3.x; break;						 
							case 4:loc_val_2[X].out = loc_val_2[X].out  *1 +tracking4.x; break;
							case 5:loc_val_2[X].out = loc_val_2[X].out  *1 +tracking5.x; break;
					  	case 6:loc_val_2[X].out = loc_val_2[X].out  *1 +tracking6.x; break;
						}
						
		        if(loc_val_2[X].out>15.0f)	loc_val_2[X].out=15.0f;
					  if(loc_val_2[X].out<-15.0f)	loc_val_2[X].out=-15.0f;   
						
						loc_val_2[Y].out = -loc_val_2[Y].out *1;
					  if(loc_val_2[Y].out>15.0f)	loc_val_2[Y].out=15.0f;
					  if(loc_val_2[Y].out<-15.0f)	loc_val_2[Y].out=-15.0f;   
				
				Vel_X_Out = loc_val_2[X].out;
				Vel_Y_Out = loc_val_2[Y].out;		
 	}
	
	if(my_flight.mode == Follow_Line_Mode)    //��ֱ������
	{
		 
	       ur_loc_c.exp[X] = -45;    // 
			   ur_loc_c.exp[Y] = -25;

						
				if(dot.ok == 1)   //��ͷ����Ϊx��
				{			 				
					ur_loc_c.fb[X] = mv->fix_y;     //�϶��½����
					ur_loc_c.fb[Y] = mv->fix_x;

//					ur_loc_c.fb[X] = 0;     //�϶��½����
//					ur_loc_c.fb[Y] = mv->fix_x;
					
//										ur_loc_c.fb[X] = mv->fix_y;     //�϶��½����
//					ur_loc_c.fb[Y] = 0;
					
				}
					
				else
				{
					ur_loc_c.fb[X] = -45;      //���㷴����ֵ//�϶��½����
					ur_loc_c.fb[Y] = -25;
				}
				
				PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
											0 ,				//ǰ��ֵ
											ur_loc_c.exp[X] ,				//����ֵ���趨ֵ��
											ur_loc_c.fb[X] ,			//����ֵ���� 
											&loc_arg_2[X], //PID�����ṹ��
											&loc_val_2[X],	//PID���ݽṹ��
											50,//��������޷�
											10 	*flag_trackingoff		//integration limit�������޷�
											);

				PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
											0 ,				//ǰ��ֵ
											ur_loc_c.exp[Y] ,				//����ֵ���趨ֵ��
											ur_loc_c.fb[Y] ,			//����ֵ���� 
											&loc_arg_2[Y], //PID�����ṹ��
											&loc_val_2[Y],	//PID���ݽṹ��
											50,//��������޷�
											10 	*flag_trackingoff		//integration limit�������޷�
											);						
													
					  loc_val_2[X].out = loc_val_2[X].out  *1;
		        if(loc_val_2[X].out>15.0f)	loc_val_2[X].out=15.0f;
					  if(loc_val_2[X].out<-15.0f)	loc_val_2[X].out=-15.0f;   
						
						loc_val_2[Y].out = -loc_val_2[Y].out *1;
					  if(loc_val_2[Y].out>15.0f)	loc_val_2[Y].out=15.0f;
					  if(loc_val_2[Y].out<-15.0f)	loc_val_2[Y].out=-15.0f;   
				
				Vel_X_Out = loc_val_2[X].out;
				Vel_Y_Out = loc_val_2[Y].out;		
				
	}
	
	
	if(contral_mode == 1)   //xy�Ὺ���ɱ�־λ
	{
		if(Pro_Ctr_step ==1)
		{
				Vel_X_Out = tracking1.x;
				Vel_Y_Out = tracking1.y;	
		}

	
		if(Pro_Ctr_step ==2)
		{
				Vel_X_Out = tracking2.x;
				Vel_Y_Out = tracking2.y;	
		}

		if(Pro_Ctr_step ==3)
		{
				Vel_X_Out = tracking3.x;
				Vel_Y_Out = tracking3.y;	
		}

		if(Pro_Ctr_step ==4)
		{
				Vel_X_Out = tracking4.x;
				Vel_Y_Out = tracking4.y;	
		}

		if(Pro_Ctr_step ==5)
		{
				Vel_X_Out = tracking5.x;
				Vel_Y_Out = tracking5.y;	
		}

		if(Pro_Ctr_step ==6)
		{
				Vel_X_Out = tracking6.x;
				Vel_Y_Out = tracking6.y;	
		}	
	}
		
}

void Walk_Ctrl(u8 dir,float speed)
{
	speed = ABS(speed);
	switch(dir)
	{
		case S:
			tracking1.x = 0;
		  tracking2.x=0;
			tracking1.y = 0;
		  tracking2.y =0;
		  tracking3.x=0;
		 tracking3.y =0;
		  tracking4.x=0;
		 tracking4.y =0;		
			  tracking5.x=0;
		 tracking5.y =0;
		  tracking6.x=0;
		 tracking6.y =0;		
		  
		break;
		
		case F:		
			tracking1.x = speed;
		  tracking2.x = speed;
		
			tracking1.y = 0;
			tracking2.y = 0;
		
		   tracking3.x = speed;
		  tracking3.y =0;
		
			tracking4.x = speed;
		  tracking4.y =0;
		
		  tracking5.x = speed;
		  tracking5.y =0;
		
			tracking6.x = speed;
		  tracking6.y =0;
		break;

		
		case B:		
			tracking1.x = -speed;
		  tracking2.x = -speed;
			tracking1.y = 0;
		  tracking2.y = 0;
		
		  tracking3.x = -speed;
		  tracking3.y = 0;
		
		  tracking4.x = -speed;
		  tracking4.y = 0;

		  tracking5.x = -speed;
		  tracking5.y = 0;

		  tracking6.x = -speed;
		  tracking6.y = 0;		
		
		break;
		
	 case R:	
			tracking1.x = 0;
	    tracking2.x = 0;
			tracking1.y = -speed;
			tracking2.y = -speed;	


      tracking3.x = 0;
      tracking3.y= -speed;

	    tracking4.x = 0;
      tracking4.y= -speed;
	 
	 
	    tracking5.x = 0;
      tracking5.y= -speed;
	 
	    tracking6.x = 0;
      tracking6.y= -speed;
	 
		break;
	 
	 	case L:	
			tracking1.x = 0;
			tracking2.x = 0;
			tracking1.y = speed;
			tracking2.y = speed;
		
			tracking3.x = 0;
			tracking3.y = speed;

			tracking4.x = 0;
			tracking4.y = speed;

			tracking5.x = 0;
			tracking5.y = speed;

			tracking6.x = 0;
			tracking6.y = speed;		
		break;
	}
}

extern u8 Pro_Ctr_step1;
extern u8 Pro_Ctr_step2;
extern u8 Pro_Ctr_step;
u8 YAW_flag;
u8 Dot_flag;
extern u8 laser_flag;
extern u8 laser_TX;
extern  u8 User_Alt_ctrl;

void Tracking_Ctrl1(float dT)    //  ������ ��� 17 16 14 18 ��ά�� 1
{
    tracking1.speed = TRACKING_SPEED; //�ٶȸ�ֵ
	
		if(Pro_Ctr_step != 1)
		{
			tracking1.time = 0;
			tracking1.step = 0;	
			return;
		}
		
		//��ʱ
		tracking1.time += dT;
		
	switch(tracking1.step)
	{
	     //�̿� 
		case 0:
			if( tracking1.time>=3)   //�������
			{							
		     FC_Unlock();
				 tracking1.time=0;
				 tracking1.step = 1; 
			}
			else
			{	
			     my_flight.mode = Follow_Null;
				   my_flight.color = NUL;
			}
		  break;	
			
		case 1:
			  if( tracking1.time >2.0f)		
				{
					 OneKey_Takeoff(145);			
					 tracking1.step = 2;
					 tracking1.time = 0;
				}
		  break;

		case 2:
			if( tracking1.time>=2.0f || ano_of.of_alt_cm>135 )   //�ȴ��ﵽ�߶�
			{	
				  tracking1.step = 3;
				  tracking1.time = 0;	
          contral_mode = 1;
  		}
		  break;
			
		case 3:
			 if(tracking1.time<9.5f)
			 {
			   Walk_Ctrl(F, tracking1.speed);//��ǰ�ɣ�Զ����ɵ�
			 }
			 
			 else 
			 {
				 tracking1.step = 4;   //���λ��
			   tracking1.time = 0; 			  
 				 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 contral_mode =0;
			 }
		  break;
			
		case 4:
			 if(dot.ok)
			{			
				Walk_Ctrl(S,tracking1.speed);	 //�ٶȽ���PID����
				if(tracking1.time >6.0f)       //2s�� Ӧ���ȶ�
				{
				  tracking1.time = 0;
				  tracking1.step = 5;
					contral_mode=1;  //����������    x,y ������� 				
					my_flight.mode  = Follow_Null;
		      my_flight.color = NUL;	
				} 
			}
			else	
			{
				Walk_Ctrl(F, tracking1.speed);//��ǰ Ѳ���
			  tracking1.time =0;
			}
		break;
			
			
		case 5:    //����������
		   if(tracking1.time <2.5f)		    Walk_Ctrl(R, tracking1.speed);
			 
		   if(tracking1.time == 2.5f)     Walk_Ctrl(S, tracking1.speed);
	
       if(tracking1.time >2.5f && tracking1.time <3.0f)   Walk_Ctrl(F, tracking1.speed);  
		
			 if(tracking1.time >=3.0f)
			 {
				  Walk_Ctrl(S, tracking1.speed);
			    tracking1.time  =0;
				  tracking1.step =6;
				  laser_flag=1;
			 }
			 break;

		case 6:
			if(laser_TX == 1)   //A������������  ��ʼ28����
			{			
				if(tracking1.time <2.9f)   Walk_Ctrl(F, tracking1.speed); //��ǰ��  50cm    15*3.33=49.95 
				if(tracking1.time >2.9f && tracking1.time < 3.1f)  Walk_Ctrl(S, tracking1.speed);   //��һ��
				if(tracking1.time >3.1f  && tracking1.time < 4.3f)  Walk_Ctrl(R,LOW_SPEED);   
				if(tracking1.time >4.3f) 
				{
					Walk_Ctrl(S, tracking1.speed);
				  laser_flag = 1;
					laser_TX = 0;
					tracking1.time = 0;
					tracking1.step = 7;
				}
			}
			else tracking1.time = 0;
			break;
			
		case 7:
			if(laser_TX == 1)   //28������������  ��ʼ27����
			{						
				if(tracking1.time <3.33f)   Walk_Ctrl(R, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking1.time >3.33f && tracking1.time <3.8f)  Walk_Ctrl(S, tracking1.speed);   //��һ��
				if(tracking1.time >3.8f  && tracking1.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking1.time >5.0f)
				{
					Walk_Ctrl(S, tracking1.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking1.time = 0;
					tracking1.step = 8;
				}
			}
			else tracking1.time = 0;
		 break;
		
		case 8:
			if(laser_TX == 1)   //27������������  ��ʼ26����
			{						
			  if(tracking1.time <3.33f)   Walk_Ctrl(R, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking1.time >3.33f && tracking1.time <3.6f)  Walk_Ctrl(S, tracking1.speed);   //��һ��
				if(tracking1.time >3.6f && tracking1.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking1.time >5.0f)
				{
					Walk_Ctrl(S, tracking1.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking1.time = 0;
					tracking1.step = 9;
				}
			}
			else tracking1.time = 0;
		 break;
			
			
//		case 9:
//			if(laser_TX == 1)   //27������������  ��ʼ26����
//			{						
//				if(tracking1.time <3.33f)   Walk_Ctrl(R, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
//				else 
//				{
//				  laser_flag = 1;
//					laser_TX = 0;
//					tracking1.time = 0;
//					tracking1.step = 10;
//				}
//			}
//			else tracking1.time = 0;
//		 break;			
		
		case 9:
			if(laser_TX == 1)   //26������������  ��ʼ25����
			{						
				if(tracking1.time <3.7f)   Walk_Ctrl(R, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking1.time >3.7f && tracking1.time <3.9f)  Walk_Ctrl(S, tracking1.speed);   //��һ��
				if(tracking1.time >3.9f && tracking1.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking1.time >5.0f)
				{
					Walk_Ctrl(S, tracking1.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking1.time = 0;
					tracking1.step = 10;
				}
			}
			else tracking1.time = 0;
		 break;	

		case 10:
			if(laser_TX == 1)   //25������������   ��ʼ24����
			{						
				if(tracking1.time <3.7f)   Walk_Ctrl(R, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking1.time >3.7f && tracking1.time <4.0f)  Walk_Ctrl(S, tracking1.speed);   //��һ��
				if(tracking1.time >4.0f  && tracking1.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking1.time >5.0f)
				{
					Walk_Ctrl(S, tracking1.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking1.time = 0;
					tracking1.step = 11;
				}
			}
			else tracking1.time = 0;
		 break;
			
//	  case 11:
//		if(laser_TX == 1)   //25������������   ��ʼ24����
//		{						
//			if(tracking1.time <3.33f)   Walk_Ctrl(R, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
//			else 
//			{
//				Walk_Ctrl(S, tracking1.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking1.time = 0;
//				tracking1.step = 12;
//			}
//		}
//		else tracking1.time = 0;
//	 break;
//		
		case 11:
		if(laser_TX == 1)   //24������������   17����  ����
		{						
			if(tracking1.time <3.33f)   Walk_Ctrl(B, tracking1.speed); //�����  50cm    15*3.33=49.95 
			if(tracking1.time >3.33f && tracking1.time <3.6f)  Walk_Ctrl(S, tracking1.speed);   //��һ��
			if(tracking1.time >3.6f  && tracking1.time <4.7f)  Walk_Ctrl(R, LOW_SPEED);   
			if(tracking1.time >4.7f) 
			{
				
				 Walk_Ctrl(S, tracking1.speed); 
				 if(tracking1.time >5.7f)
				 {
				   tracking1.time = 0;
				   tracking1.step = 13;				 
				 }
				//laser_flag = 1;
				//laser_TX = 0;
			}
		}
		else tracking1.time = 0;
	 break;
		
		case 13:
		if(laser_TX == 1)   //17������������  	16����
		{						
			if(tracking1.time <3.33f)   Walk_Ctrl(R, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
//			if(tracking1.time >3.33f&&  tracking1.time <4.3f)  Walk_Ctrl(S, tracking1.speed);
//			if(tracking1.time > 4.3f)
			else
			{
				Walk_Ctrl(S, tracking1.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step = 14;
			}
		}
		else tracking1.time = 0;
	 break;		

		case 14:
		if(laser_TX == 1)   //16������������   ��ʼ23����
		{						
			if(tracking1.time <3.3f)   Walk_Ctrl(F, tracking1.speed); //��ǰ��  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step = 15;
			}
		}
		else tracking1.time = 0;
	 break;		

		case 15:
		if(laser_TX == 1)   //23������������  
		{						
			if(tracking1.time <3.3f)   Walk_Ctrl(R, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S,tracking1.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step = 50;   
				my_flight.mode = Follow_Line_Mode;  //������ͷ��ֱ��
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking1.time = 0;
	 break;	
		
		case 50:    //��ֱ�� ���ȶ�һ��

		
					 if(tracking1.time <7.0f) //Ĭ���ȶ�
					 {
							Walk_Ctrl(S, 0); 
					 }
					 else
					 {
						my_flight.mode = Follow_Null;	   
						my_flight.color = NUL;
						contral_mode = 1;           //  ��ʼ����
						laser_flag = 1;
						laser_TX = 0;
						tracking1.time = 0;
						tracking1.step = 16;	 
					 }
					 
	
    break;	
			 
			 
		case 16:
		if(laser_TX == 1)   //22������������   ��ʼ15����
		{						
			if(tracking1.time <3.0f)   Walk_Ctrl(B, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;	
		
		case 17:
		if(laser_TX == 1)   //15������������   ��ʼ11����
		{						
			if(tracking1.time <3.0f)   Walk_Ctrl(B, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking1.time >3.0f && tracking1.time <3.4f)   Walk_Ctrl(S, tracking1.speed);
			if(tracking1.time >3.4f  && tracking1.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			if(tracking1.time >5.0f)
			{
				Walk_Ctrl(S, tracking1.speed);  
				  if(tracking1.time>6.0f)
					{
						laser_flag = 1;
						laser_TX = 0;
						tracking1.time = 0;
						tracking1.step ++;
					}
			}
		}
		else tracking1.time = 0;
	 break;
		
		case 18:
					if(laser_TX == 1)   //11������������   ��ʼ12����
		{	
			if(tracking1.time <3.33f)   Walk_Ctrl(L, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking1.time >3.33f &&tracking1.time <3.6f)   Walk_Ctrl(S, tracking1.speed);
			if(tracking1.time >3.6f  && tracking1.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			//if(tracking1.time >5.0f  && tracking1.time <6.0f)    Walk_Ctrl(F, LOW_SPEED);			
			if(tracking1.time >5.0f) 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;
		
		case 19:
		if(laser_TX == 1)   //12������������   ��ʼ13����
		{						
			if(tracking1.time <3.33f)   Walk_Ctrl(L, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking1.time >3.33f && tracking1.time <3.6f)     Walk_Ctrl(S, tracking1.speed);
			if(tracking1.time >3.6f  && tracking1.time <4.2f)     Walk_Ctrl(R, tracking1.speed);
			if(tracking1.time >4.2f  && tracking1 .time <5.7f)   Walk_Ctrl(F, LOW_SPEED);
			if(tracking1.time >5.7f) 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step =20;
			}
		}
		else tracking1.time = 0;
	 break;	
		
		
//		
//		case 52:    //��ֱ�� ���ȶ�һ��
//	  
//		  if(tracking1.time < 5.0f)
//			{
//			  Yaw_Dps_Out = 5;
//			}
//	  else 
//	  {
//			  tracking1.time = 0;
//			  laser_flag = 1;
//				laser_TX = 0;
//			  tracking1.step =20;
//      }	
//		}			
//    break;	
		
		case 20:
		if(laser_TX == 1)   //13������������   ��ʼ9����
		{						
			if(tracking1.time <3.7f)   Walk_Ctrl(B, tracking1.speed); //�����  50cm    15*3.33=49.95 
			
			if(tracking1.time >3.7f && tracking1.time <3.90f)    Walk_Ctrl(S, tracking1.speed); 
			
			if(tracking1.time >3.9f &&  tracking1.time <4.7f)     Walk_Ctrl(R, LOW_SPEED);
			
      if(tracking1.time >4.7f)			
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;			
		
		case 21:
		if(laser_TX == 1)   //9������������   ��ʼ7����
		{						
			if(tracking1.time <3.9f)   Walk_Ctrl(B, tracking1.speed); //�����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;	

		case 22:
		if(laser_TX == 1)   //7������������   ��ʼ6����
		{						
			if(tracking1.time <3.6f)   Walk_Ctrl(R, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;

		case 23:
		if(laser_TX == 1)   //6������������   ��ʼ5����
		{						
			if(tracking1.time <3.33f)   Walk_Ctrl(R, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking1.time >3.33f && tracking1.time <3.6f)  Walk_Ctrl(S, tracking1.speed);
			if(tracking1.time >3.6f &&  tracking1.time <4.8f)  Walk_Ctrl(B, LOW_SPEED);
			if(tracking1.time > 4.8f)
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;		

		case 24:
		if(laser_TX == 1)   //5������������   ��ʼ1����
		{						
			if(tracking1.time <3.1f)   Walk_Ctrl(B, tracking1.speed); //����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step =25;
//				my_flight.mode = Follow_Line_Mode;
//				contral_mode = 0;
			}
		}
		else tracking1.time = 0;
	 break;

//		case 51:    //��ֱ�� ���ȶ�һ��
//			
//	     cross_step = 2;

////	  if(dot.ok == 1)
////	  {
//			 if(tracking1.time < 6.0f) //Ĭ���ȶ�
//			 {
//					Walk_Ctrl(S, tracking1.speed); 
//			 }
//			 else
//			 {
//				my_flight.mode = Follow_Null;
//				contral_mode = 1;           //  ��ʼ����
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking1.time = 0;
//				tracking1.step = 25;	 
//			 }
////	  }
////	 else 
////	 {
////			tracking1.time -= dT;
////	 }		
//    break;	
//			 
		
		
		case 25:
		if(laser_TX == 1)   //1������������   ��ʼ2����
		{						
			if(tracking1.time <3.3f)   Walk_Ctrl(L,tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;	
		
		case 26:
		if(laser_TX == 1)   //2������������   ��ʼ3����
		{						
			if(tracking1.time <3.1f)   Walk_Ctrl(L, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;			
	
		case 27:
		if(laser_TX == 1)   //3������������   ��ʼ4����
		{						
			if(tracking1.time <3.0f)   Walk_Ctrl(L, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;		

		case 28:
		if(laser_TX == 1)   //4������������   ��ʼ8����
		{						
			if(tracking1.time <3.0f)   Walk_Ctrl(F, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;	
		
	

		case 29:
		if(laser_TX == 1)   //8������������   ��ʼ10����
		{						
			if(tracking1.time <3.0f)   Walk_Ctrl(F, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;	

		case 30:
		if(laser_TX == 1)   //10������������   ��14����
		{						
			if(tracking1.time <3.0f)   Walk_Ctrl(F, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;	


		case 31:
		if(laser_TX == 1)   //14������������  ��18����
		{						
			if(tracking1.time <3.0f)   Walk_Ctrl(F, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;	

		
		case 32:
		if(laser_TX == 1)   //18������������   ��ʼ19����
		{						
			if(tracking1.time <3.33f)   Walk_Ctrl(L, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;	

		
		case 33:
		if(laser_TX == 1)   //19������������   ��ʼ20����
		{						
			if(tracking1.time <3.33f)   Walk_Ctrl(L, tracking1.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking1.time >3.33f && tracking1.time <3.6f)  Walk_Ctrl(S,tracking1.speed);
			if(tracking1.time >3.6f  && tracking1.time <4.5f)  Walk_Ctrl(F,LOW_SPEED);
			if(tracking1.time >4.5f) 
			{
				Walk_Ctrl(S, tracking1.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
			}
		}
		else tracking1.time = 0;
	 break;	

		
	case 34:
		if(laser_TX == 1)   //20������������  ��ʼ����
		{						
			if(tracking1.time <5.0f)   Walk_Ctrl(L, tracking1.speed); //���ҷ�  50cm    15*4.0=60
			else 
			{ 
				laser_TX = 0;
				tracking1.time = 0;
				tracking1.step ++;
				my_flight.mode=Follow_Dot_Mode;
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking1.time = 0;
	 break;	
		
		
	 case 35:
		 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking1.speed);
			 
			  if(tracking1.time >4.0f)
				{
				   tracking1.step = 36;
					 tracking1.time = 0;      	
					 my_flight.mode=Follow_Null;
				   contral_mode = 1;
				}
		 }
		 
		 else 
		 {
			 //Walk_Ctrl(B,tracking1.speed);
			 tracking1.time = 0;
		 }
		 break;	

	 case 36:
	   if(tracking1.time <2.0f) Walk_Ctrl(L,tracking1.speed);
	   if(tracking1.time >2.0f && tracking1.time <15.5f)  Walk_Ctrl(B,tracking1.speed);
     if(tracking1.time  >=10.0f )   Car_Send(1);	  

	   if(tracking1.time >=15.5f)
		 {
			  tracking1.step = 37;
			  tracking1.time = 0;
			 
			 	 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 dot_two_mode=1;
				 contral_mode = 0;	
		 }
		 break;
		 
	 case 37:
			 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking1.speed);
			 
			  if(tracking1.time >3.0f)
				{
				   tracking1.step = 99;
					 tracking1.time = 0; 
           dot_two_mode=0;					
				}
		 }
		 
		 else 
		{
					 tracking1.step = 99;
					 tracking1.time = 0; 
		 }	
		break;
		
		case 99:
			 tracking1.step += OneKey_Land();
		   tracking1.time =0;
			break;
				
		case 100:
			 if( tracking1.time >4.0f)
			 {
				  tracking1.step = 0;
				  tracking1.time =0;
				 FC_Lock();  //����
				 Pro_Ctr_step =0;
			 }
			 break;
  }
}

void Tracking_Ctrl2(float dT)    //  ������ ��� 17 16 14 18 ��ά�� 2
{
    tracking2.speed = TRACKING_SPEED; //�ٶȸ�ֵ
	
		if(Pro_Ctr_step != 2)
		{
			tracking2.time = 0;
			tracking2.step = 0;	
			return;
		}
		
		//��ʱ
		tracking2.time += dT;
		
	switch(tracking2.step)
	{
	     //�̿� 
		case 0:
			if( tracking2.time>=3)   //�������
			{							
		     FC_Unlock();
				 tracking2.time=0;
				 tracking2.step = 1; 
			}
			else
			{	
			     my_flight.mode = Follow_Null;
				   my_flight.color = NUL;
			}
		  break;	
			
		case 1:
			  if( tracking2.time >2.0f)		
				{
					 OneKey_Takeoff(145);			
					 tracking2.step = 2;
					 tracking2.time = 0;
				}
		  break;

		case 2:
			if( tracking2.time>=2.0f || ano_of.of_alt_cm>135 )   //�ȴ��ﵽ�߶�
			{	
				  tracking2.step = 3;
				  tracking2.time = 0;	
          contral_mode = 1;
  		}
		  break;
			
		case 3:
			 if(tracking2.time<9.5f)
			 {
			   Walk_Ctrl(F, tracking2.speed);//��ǰ�ɣ�Զ����ɵ�
			 }
			 
			 else 
			 {
				 tracking2.step = 4;   //���λ��
			   tracking2.time = 0; 			  
 				 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 contral_mode =0;
			 }
		  break;
			
		case 4:
			 if(dot.ok)
			{			
				Walk_Ctrl(S,tracking2.speed);	 //�ٶȽ���PID����
				if(tracking2.time >6.0f)       //2s�� Ӧ���ȶ�
				{
				  tracking2.time = 0;
				  tracking2.step = 5;
					contral_mode=1;  //����������    x,y ������� 				
					my_flight.mode  = Follow_Null;
		      my_flight.color = NUL;	
				} 
			}
			else	
			{
				Walk_Ctrl(F, tracking2.speed);//��ǰ Ѳ���
			  tracking2.time =0;
			}
		break;
			
			
		case 5:    //����������
		   if(tracking2.time <2.5f)		    Walk_Ctrl(R, tracking2.speed);
			 
		   if(tracking2.time == 2.5f)     Walk_Ctrl(S, tracking2.speed);
	
       if(tracking2.time >2.5f && tracking2.time <3.0f)   Walk_Ctrl(F, tracking2.speed);  
		
			 if(tracking2.time >=3.0f)
			 {
				  Walk_Ctrl(S, tracking2.speed);
			    tracking2.time  =0;
				  tracking2.step =6;
				  laser_flag=1;
			 }
			 break;

		case 6:
			if(laser_TX == 1)   //A������������  ��ʼ28����
			{			
				if(tracking2.time <2.9f)   Walk_Ctrl(F, tracking2.speed); //��ǰ��  50cm    15*3.33=49.95 
				if(tracking2.time >2.9f && tracking2.time < 3.1f)  Walk_Ctrl(S, tracking2.speed);   //��һ��
				if(tracking2.time >3.1f  && tracking2.time < 4.3f)  Walk_Ctrl(R,LOW_SPEED);   
				if(tracking2.time >4.3f) 
				{
					Walk_Ctrl(S, tracking2.speed);
				  laser_flag = 1;
					laser_TX = 0;
					tracking2.time = 0;
					tracking2.step = 7;
				}
			}
			else tracking2.time = 0;
			break;
			
		case 7:
			if(laser_TX == 1)   //28������������  ��ʼ27����
			{						
				if(tracking2.time <3.33f)   Walk_Ctrl(R, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking2.time >3.33f && tracking2.time <3.8f)  Walk_Ctrl(S, tracking2.speed);   //��һ��
				if(tracking2.time >3.8f  && tracking2.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking2.time >5.0f)
				{
					Walk_Ctrl(S, tracking2.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking2.time = 0;
					tracking2.step = 8;
				}
			}
			else tracking2.time = 0;
		 break;
		
		case 8:
			if(laser_TX == 1)   //27������������  ��ʼ26����
			{						
			  if(tracking2.time <3.33f)   Walk_Ctrl(R, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking2.time >3.33f && tracking2.time <3.6f)  Walk_Ctrl(S, tracking2.speed);   //��һ��
				if(tracking2.time >3.6f && tracking2.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking2.time >5.0f)
				{
					Walk_Ctrl(S, tracking2.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking2.time = 0;
					tracking2.step = 9;
				}
			}
			else tracking2.time = 0;
		 break;
			
			
//		case 9:
//			if(laser_TX == 1)   //27������������  ��ʼ26����
//			{						
//				if(tracking2.time <3.33f)   Walk_Ctrl(R, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
//				else 
//				{
//				  laser_flag = 1;
//					laser_TX = 0;
//					tracking2.time = 0;
//					tracking2.step = 10;
//				}
//			}
//			else tracking2.time = 0;
//		 break;			
		
		case 9:
			if(laser_TX == 1)   //26������������  ��ʼ25����
			{						
				if(tracking2.time <3.7f)   Walk_Ctrl(R, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking2.time >3.7f && tracking2.time <3.9f)  Walk_Ctrl(S, tracking2.speed);   //��һ��
				if(tracking2.time >3.9f && tracking2.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking2.time >5.0f)
				{
					Walk_Ctrl(S, tracking2.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking2.time = 0;
					tracking2.step = 10;
				}
			}
			else tracking2.time = 0;
		 break;	

		case 10:
			if(laser_TX == 1)   //25������������   ��ʼ24����
			{						
				if(tracking2.time <3.7f)   Walk_Ctrl(R, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking2.time >3.7f && tracking2.time <4.0f)  Walk_Ctrl(S, tracking2.speed);   //��һ��
				if(tracking2.time >4.0f  && tracking2.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking2.time >5.0f)
				{
					Walk_Ctrl(S, tracking2.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking2.time = 0;
					tracking2.step = 11;
				}
			}
			else tracking2.time = 0;
		 break;
			
//	  case 11:
//		if(laser_TX == 1)   //25������������   ��ʼ24����
//		{						
//			if(tracking2.time <3.33f)   Walk_Ctrl(R, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
//			else 
//			{
//				Walk_Ctrl(S, tracking2.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking2.time = 0;
//				tracking2.step = 12;
//			}
//		}
//		else tracking2.time = 0;
//	 break;
//		
		case 11:
		if(laser_TX == 1)   //24������������   17����  ����
		{						
			if(tracking2.time <3.33f)   Walk_Ctrl(B, tracking2.speed); //�����  50cm    15*3.33=49.95 
			if(tracking2.time >3.33f && tracking2.time <3.6f)  Walk_Ctrl(S, tracking2.speed);   //��һ��
			if(tracking2.time >3.6f  && tracking2.time <4.7f)  Walk_Ctrl(R, LOW_SPEED);   
			if(tracking2.time >4.7f) 
			{
				
				 Walk_Ctrl(S, tracking2.speed); 
				 if(tracking2.time >5.7f)
				 {
				   tracking2.time = 0;
				   tracking2.step = 13;				 
				 }
				//laser_flag = 1;
				//laser_TX = 0;
			}
		}
		else tracking2.time = 0;
	 break;
		
		case 13:
		if(laser_TX == 1)   //17������������  	16����
		{						
			if(tracking2.time <3.33f)   Walk_Ctrl(R, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
//			if(tracking2.time >3.33f&&  tracking2.time <4.3f)  Walk_Ctrl(S, tracking2.speed);
//			if(tracking2.time > 4.3f)
			else
			{
				Walk_Ctrl(S, tracking2.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step = 14;
			}
		}
		else tracking2.time = 0;
	 break;		

		case 14:
		if(laser_TX == 1)   //16������������   ��ʼ23����
		{						
			if(tracking2.time <3.3f)   Walk_Ctrl(F, tracking2.speed); //��ǰ��  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step = 15;
			}
		}
		else tracking2.time = 0;
	 break;		

		case 15:
		if(laser_TX == 1)   //23������������  
		{						
			if(tracking2.time <3.3f)   Walk_Ctrl(R, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S,tracking2.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step = 50;   
				my_flight.mode = Follow_Line_Mode;  //������ͷ��ֱ��
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking2.time = 0;
	 break;	
		
		case 50:    //��ֱ�� ���ȶ�һ��

		
					 if(tracking2.time <7.0f) //Ĭ���ȶ�
					 {
							Walk_Ctrl(S, 0); 
					 }
					 else
					 {
						my_flight.mode = Follow_Null;	   
						my_flight.color = NUL;
						contral_mode = 1;           //  ��ʼ����
						laser_flag = 1;
						laser_TX = 0;
						tracking2.time = 0;
						tracking2.step = 16;	 
					 }
					 
	
    break;	
			 
			 
		case 16:
		if(laser_TX == 1)   //22������������   ��ʼ15����
		{						
			if(tracking2.time <3.0f)   Walk_Ctrl(B, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;	
		
		case 17:
		if(laser_TX == 1)   //15������������   ��ʼ11����
		{						
			if(tracking2.time <3.0f)   Walk_Ctrl(B, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking2.time >3.0f && tracking2.time <3.4f)   Walk_Ctrl(S, tracking2.speed);
			if(tracking2.time >3.4f  && tracking2.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			if(tracking2.time >5.0f)
			{
				Walk_Ctrl(S, tracking2.speed);  
				  if(tracking2.time>6.0f)
					{
						laser_flag = 1;
						laser_TX = 0;
						tracking2.time = 0;
						tracking2.step ++;
					}
			}
		}
		else tracking2.time = 0;
	 break;
		
		case 18:
					if(laser_TX == 1)   //11������������   ��ʼ12����
		{	
			if(tracking2.time <3.33f)   Walk_Ctrl(L, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking2.time >3.33f &&tracking2.time <3.6f)   Walk_Ctrl(S, tracking2.speed);
			if(tracking2.time >3.6f  && tracking2.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			//if(tracking2.time >5.0f  && tracking2.time <6.0f)    Walk_Ctrl(F, LOW_SPEED);			
			if(tracking2.time >5.0f) 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;
		
		case 19:
		if(laser_TX == 1)   //12������������   ��ʼ13����
		{						
			if(tracking2.time <3.33f)   Walk_Ctrl(L, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking2.time >3.33f && tracking2.time <3.6f)     Walk_Ctrl(S, tracking2.speed);
			if(tracking2.time >3.6f  && tracking2.time <4.2f)     Walk_Ctrl(R, tracking2.speed);
			if(tracking2.time >4.2f  && tracking2 .time <5.7f)   Walk_Ctrl(F, LOW_SPEED);
			if(tracking2.time >5.7f) 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step =20;
			}
		}
		else tracking2.time = 0;
	 break;	
		
		
//		
//		case 52:    //��ֱ�� ���ȶ�һ��
//	  
//		  if(tracking2.time < 5.0f)
//			{
//			  Yaw_Dps_Out = 5;
//			}
//	  else 
//	  {
//			  tracking2.time = 0;
//			  laser_flag = 1;
//				laser_TX = 0;
//			  tracking2.step =20;
//      }	
//		}			
//    break;	
		
		case 20:
		if(laser_TX == 1)   //13������������   ��ʼ9����
		{						
			if(tracking2.time <3.7f)   Walk_Ctrl(B, tracking2.speed); //�����  50cm    15*3.33=49.95 
			
			if(tracking2.time >3.7f && tracking2.time <3.90f)    Walk_Ctrl(S, tracking2.speed); 
			
			if(tracking2.time >3.9f &&  tracking2.time <4.7f)     Walk_Ctrl(R, LOW_SPEED);
			
      if(tracking2.time >4.7f)			
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;			
		
		case 21:
		if(laser_TX == 1)   //9������������   ��ʼ7����
		{						
			if(tracking2.time <3.9f)   Walk_Ctrl(B, tracking2.speed); //�����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;	

		case 22:
		if(laser_TX == 1)   //7������������   ��ʼ6����
		{						
			if(tracking2.time <3.6f)   Walk_Ctrl(R, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;

		case 23:
		if(laser_TX == 1)   //6������������   ��ʼ5����
		{						
			if(tracking2.time <3.33f)   Walk_Ctrl(R, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking2.time >3.33f && tracking2.time <3.6f)  Walk_Ctrl(S, tracking2.speed);
			if(tracking2.time >3.6f &&  tracking2.time <4.8f)  Walk_Ctrl(B, LOW_SPEED);
			if(tracking2.time > 4.8f)
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;		

		case 24:
		if(laser_TX == 1)   //5������������   ��ʼ1����
		{						
			if(tracking2.time <3.1f)   Walk_Ctrl(B, tracking2.speed); //����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step =25;
//				my_flight.mode = Follow_Line_Mode;
//				contral_mode = 0;
			}
		}
		else tracking2.time = 0;
	 break;

//		case 51:    //��ֱ�� ���ȶ�һ��
//			
//	     cross_step = 2;

////	  if(dot.ok == 1)
////	  {
//			 if(tracking2.time < 6.0f) //Ĭ���ȶ�
//			 {
//					Walk_Ctrl(S, tracking2.speed); 
//			 }
//			 else
//			 {
//				my_flight.mode = Follow_Null;
//				contral_mode = 1;           //  ��ʼ����
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking2.time = 0;
//				tracking2.step = 25;	 
//			 }
////	  }
////	 else 
////	 {
////			tracking2.time -= dT;
////	 }		
//    break;	
//			 
		
		
		case 25:
		if(laser_TX == 1)   //1������������   ��ʼ2����
		{						
			if(tracking2.time <3.3f)   Walk_Ctrl(L,tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;	
		
		case 26:
		if(laser_TX == 1)   //2������������   ��ʼ3����
		{						
			if(tracking2.time <3.1f)   Walk_Ctrl(L, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;			
	
		case 27:
		if(laser_TX == 1)   //3������������   ��ʼ4����
		{						
			if(tracking2.time <3.0f)   Walk_Ctrl(L, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;		

		case 28:
		if(laser_TX == 1)   //4������������   ��ʼ8����
		{						
			if(tracking2.time <3.0f)   Walk_Ctrl(F, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;	
		
	

		case 29:
		if(laser_TX == 1)   //8������������   ��ʼ10����
		{						
			if(tracking2.time <3.0f)   Walk_Ctrl(F, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;	

		case 30:
		if(laser_TX == 1)   //10������������   ��14����
		{						
			if(tracking2.time <3.0f)   Walk_Ctrl(F, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;	


		case 31:
		if(laser_TX == 1)   //14������������  ��18����
		{						
			if(tracking2.time <3.0f)   Walk_Ctrl(F, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;	

		
		case 32:
		if(laser_TX == 1)   //18������������   ��ʼ19����
		{						
			if(tracking2.time <3.33f)   Walk_Ctrl(L, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;	

		
		case 33:
		if(laser_TX == 1)   //19������������   ��ʼ20����
		{						
			if(tracking2.time <3.33f)   Walk_Ctrl(L, tracking2.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking2.time >3.33f && tracking2.time <3.6f)  Walk_Ctrl(S,tracking2.speed);
			if(tracking2.time >3.6f  && tracking2.time <4.5f)  Walk_Ctrl(F,LOW_SPEED);
			if(tracking2.time >4.5f) 
			{
				Walk_Ctrl(S, tracking2.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
			}
		}
		else tracking2.time = 0;
	 break;	

		
	case 34:
		if(laser_TX == 1)   //20������������  ��ʼ����
		{						
			if(tracking2.time <5.0f)   Walk_Ctrl(L, tracking2.speed); //���ҷ�  50cm    15*4.0=60
			else 
			{ 
				laser_TX = 0;
				tracking2.time = 0;
				tracking2.step ++;
				my_flight.mode=Follow_Dot_Mode;
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking2.time = 0;
	 break;	
		
		
	 case 35:
		 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking2.speed);
			 
			  if(tracking2.time >4.0f)
				{
				   tracking2.step = 36;
					 tracking2.time = 0;      	
					 my_flight.mode=Follow_Null;
				   contral_mode = 1;
				}
		 }
		 
		 else 
		 {
			 //Walk_Ctrl(B,tracking2.speed);
			 tracking2.time = 0;
		 }
		 break;	

	 case 36:
	   if(tracking2.time <2.0f) Walk_Ctrl(L,tracking2.speed);
	   if(tracking2.time >2.0f && tracking2.time <15.5f)  Walk_Ctrl(B,tracking2.speed);
     if(tracking2.time  >=10.0f )   Car_Send(2);	  

	   if(tracking2.time >=15.5f)
		 {
			  tracking2.step = 37;
			  tracking2.time = 0;
			 
			 	 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 dot_two_mode=1;
				 contral_mode = 0;	
		 }
		 break;
		 
	 case 37:
			 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking2.speed);
			 
			  if(tracking2.time >3.0f)
				{
				   tracking2.step = 99;
					 tracking2.time = 0; 
           dot_two_mode=0;					
				}
		 }
		 
		 else 
		{
					 tracking2.step = 99;
					 tracking2.time = 0; 
		 }	
		break;
		
		case 99:
			 tracking2.step += OneKey_Land();
		   tracking2.time =0;
			break;
				
		case 100:
			 if( tracking2.time >4.0f)
			 {
				  tracking2.step = 0;
				  tracking2.time =0;
				 FC_Lock();  //����
				 Pro_Ctr_step =0;
			 }
			 break;
  }
}

void Tracking_Ctrl3(float dT)    //  ������ ����  ���ǲ�ֱ�ǵ�����    Ŀǰ�ȶ���  ��������  ʡһ���� 
{
	   tracking3.speed = TRACKING_SPEED; //�ٶȸ�ֵ
	
		if(Pro_Ctr_step != 3)
		{
			tracking3.time = 0;
			tracking3.step = 0;	
			return;
		}
		
		//��ʱ
		tracking3.time += dT;
		
	switch(tracking3.step)
	{
	     //�̿� 
		case 0:
			if(tracking3.time>=3)   //�������
			{							
		     FC_Unlock();
				 tracking3.time=0;
				 tracking3.step = 1; 
			}
			else
			{	
			     my_flight.mode = Follow_Null;
				   my_flight.color = NUL;
			}
		  break;	
			
		case 1:
			  if( tracking3.time >2.0f)		
				{
					 OneKey_Takeoff(145);			
					 tracking3.step = 2;
					 tracking3.time = 0;
				}
		  break;

		case 2:
			if( tracking3.time>=2.0f || ano_of.of_alt_cm>135 )   //�ȴ��ﵽ�߶�
			{	
				  User_Alt_ctrl =1;
				  tracking3.step = 3;
				  tracking3.time = 0;	
          contral_mode = 1;
  		}
		  break;
			
		case 3:
			 if(tracking3.time<9.5f)
			 {
			   Walk_Ctrl(F, tracking3.speed);//��ǰ�ɣ�Զ����ɵ�
			 }
			 
			 else 
			 {
				 tracking3.step = 4;   //���λ��
			   tracking3.time = 0; 			  
 				 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 contral_mode =0;
			 }
		  break;
			
		case 4:
			 if(dot.ok)
			{			
				Walk_Ctrl(S,tracking3.speed);	 //�ٶȽ���PID����
				if(tracking3.time >6.0f)       //2s�� Ӧ���ȶ�
				{
				  tracking3.time = 0;
				  tracking3.step = 5;
					contral_mode=1;  //����������    x,y ������� 				
					my_flight.mode  = Follow_Null;
		      my_flight.color = NUL;	
				} 
			}
			else	
			{
				Walk_Ctrl(F, tracking3.speed);//��ǰ Ѳ���
			  tracking3.time =0;
			}
		break;
			
			
		case 5:    //����������
		   if(tracking3.time <2.5f)		    Walk_Ctrl(R, tracking3.speed);
			 
		   if(tracking3.time == 2.5f)     Walk_Ctrl(S, tracking3.speed);
	
       if(tracking3.time >2.5f && tracking3.time <3.0f)   Walk_Ctrl(F, tracking3.speed);  
		
			 if(tracking3.time >=3.0f)
			 {
				  Walk_Ctrl(S, tracking3.speed);
			    tracking3.time  =0;
				  tracking3.step =6;
				  laser_flag=1;
			 }
			 break;

		case 6:
			if(laser_TX == 1)   //A������������  ��ʼ28����
			{			
				if(tracking3.time <2.9f)   Walk_Ctrl(F, tracking3.speed); //��ǰ��  50cm    15*3.33=49.95 
				if(tracking3.time >2.9f  && tracking3.time < 3.1f)  Walk_Ctrl(S, tracking3.speed);   //��һ��
				if(tracking3.time >3.1f  && tracking3.time < 4.3f)  Walk_Ctrl(R,LOW_SPEED);   
				if(tracking3.time >4.3f) 
				{
					Walk_Ctrl(S, tracking3.speed);
				  laser_flag = 1;
					laser_TX = 0;
					tracking3.time = 0;
					tracking3.step = 7;
				}
			}
			else tracking3.time = 0;
			break;
			
		case 7:
			if(laser_TX == 1)   //28������������  ��ʼ27����
			{						
				if(tracking3.time <3.33f)   Walk_Ctrl(R, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking3.time >3.33f && tracking3.time <3.8f)  Walk_Ctrl(S, tracking3.speed);   //��һ��
				if(tracking3.time >3.8f  && tracking3.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking3.time >5.0f)
				{
					Walk_Ctrl(S, tracking3.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking3.time = 0;
					tracking3.step = 8;
				}
			}
			else tracking3.time = 0;
		 break;
		
		case 8:
			if(laser_TX == 1)   //27������������  ��ʼ26����
			{						
			  if(tracking3.time <3.33f)   Walk_Ctrl(R, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking3.time >3.33f && tracking3.time <3.6f)  Walk_Ctrl(S, tracking3.speed);   //��һ��
				if(tracking3.time >3.6f  && tracking3.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking3.time >5.0f)
				{
					Walk_Ctrl(S, tracking3.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking3.time = 0;
					tracking3.step = 9;
				}
			}
			else tracking3.time = 0;
		 break;
			
			
//		case 9:
//			if(laser_TX == 1)   //27������������  ��ʼ26����
//			{						
//				if(tracking3.time <3.33f)   Walk_Ctrl(R, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
//				else 
//				{
//				  laser_flag = 1;
//					laser_TX = 0;
//					tracking3.time = 0;
//					tracking3.step = 10;
//				}
//			}
//			else tracking3.time = 0;
//		 break;			
		
		case 9:
			if(laser_TX == 1)   //26������������  ��ʼ25����
			{						
				if(tracking3.time <3.7f)   Walk_Ctrl(R, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking3.time >3.7f && tracking3.time <3.9f)  Walk_Ctrl(S, tracking3.speed);   //��һ��
				if(tracking3.time >3.9f && tracking3.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking3.time >5.0f)
				{
					Walk_Ctrl(S, tracking3.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking3.time = 0;
					tracking3.step = 10;
				}
			}
			else tracking3.time = 0;
		 break;	

		case 10:
			if(laser_TX == 1)   //25������������   ��ʼ24����
			{						
				if(tracking3.time <3.7f)   Walk_Ctrl(R, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking3.time >3.7f && tracking3.time <4.0f)  Walk_Ctrl(S, tracking3.speed);   //��һ��
				if(tracking3.time >4.0f  && tracking3.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking3.time >5.0f)
				{
					Walk_Ctrl(S, tracking3.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking3.time = 0;
					tracking3.step = 11;
				}
			}
			else tracking3.time = 0;
		 break;
			
//	  case 11:
//		if(laser_TX == 1)   //25������������   ��ʼ24����
//		{						
//			if(tracking3.time <3.33f)   Walk_Ctrl(R, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
//			else 
//			{
//				Walk_Ctrl(S, tracking3.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking3.time = 0;
//				tracking3.step = 12;
//			}
//		}
//		else tracking3.time = 0;
//	 break;
//		
		case 11:
		if(laser_TX == 1)   //24������������   ��ʼ17����
		{						
			if(tracking3.time <3.33f)   Walk_Ctrl(B, tracking3.speed); //�����  50cm    15*3.33=49.95 
			if(tracking3.time >3.33f && tracking3.time <3.6f)  Walk_Ctrl(S, tracking3.speed);   //��һ��
			if(tracking3.time >3.6f  && tracking3.time <4.7f)  Walk_Ctrl(R, LOW_SPEED);   
			if(tracking3.time >4.7f) 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step = 13;
			}
		}
		else tracking3.time = 0;
	 break;
		
		case 13:
		if(laser_TX == 1)   //17������������   ��ʼ16����
		{						
			if(tracking3.time <3.33f)   Walk_Ctrl(R, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step = 14;
			}
		}
		else tracking3.time = 0;
	 break;		

		case 14:
		if(laser_TX == 1)   //16������������   ��ʼ23����
		{						
			if(tracking3.time <3.33f)   Walk_Ctrl(F, tracking3.speed); //��ǰ��  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step = 15;
			}
		}
		else tracking3.time = 0;
	 break;		

		case 15:
		if(laser_TX == 1)   //23������������  
		{						
			if(tracking3.time <3.3f)   Walk_Ctrl(R, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S,tracking3.speed); 
				//laser_flag = 1;
				//laser_TX = 0;
				tracking3.time = 0;
				tracking3.step = 50;
        dot.flag = 0;			
				my_flight.mode = Follow_Line_Mode;  //������ͷ��ֱ��
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking3.time = 0;
	 break;	
		
		case 50:    //��ֱ�� ���ȶ�һ��

		
					 if(tracking3.time <7.0f) //Ĭ���ȶ�
					 {
							Walk_Ctrl(S, 0); 
					 }
					 else
					 {
						my_flight.mode = Follow_Null;	   
						my_flight.color = NUL;
						contral_mode = 1;           //  ��ʼ����
						laser_flag = 1;
						laser_TX = 0;
						tracking3.time = 0;
						tracking3.step = 16;	 
					 }
					 
	
    break;	
			 
			 
		case 16:
		if(laser_TX == 1)   //22������������   ��ʼ15����
		{						
			if(tracking3.time <3.0f)   Walk_Ctrl(B, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;	
		
		case 17:
		if(laser_TX == 1)   //15������������   ��ʼ11����
		{						
			if(tracking3.time <3.0f)   Walk_Ctrl(B, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking3.time >3.0f &&tracking3.time <3.4f)   Walk_Ctrl(S, tracking3.speed);
			if(tracking3.time >3.4f  && tracking3.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			if(tracking3.time >5.0f)
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;
		
		case 18:
					if(laser_TX == 1)   //11������������   ��ʼ12����
		{	
			if(tracking3.time <3.33f)   Walk_Ctrl(L, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking3.time >3.33f &&tracking3.time <3.6f)   Walk_Ctrl(S, tracking3.speed);
			if(tracking3.time >3.6f  && tracking3.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			if(tracking3.time >5.0f) 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;
		
		case 19:
		if(laser_TX == 1)   //12������������   ��ʼ13����
		{						
			if(tracking3.time <3.33f)   Walk_Ctrl(L, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking3.time >3.33f && tracking3.time <3.6f)     Walk_Ctrl(S, tracking3.speed);
			if(tracking3.time >3.6f  && tracking3.time <4.2f)     Walk_Ctrl(R, tracking3.speed);
			if(tracking3.time >4.2f  && tracking3 .time <5.7f)   Walk_Ctrl(F, LOW_SPEED);
			if(tracking3.time >5.7f) 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step =20;
			}
		}
		else tracking3.time = 0;
	 break;	
		
		
//		
//		case 52:    //��ֱ�� ���ȶ�һ��
//	  
//		  if(tracking3.time < 5.0f)
//			{
//			  Yaw_Dps_Out = 5;
//			}
//	  else 
//	  {
//			  tracking3.time = 0;
//			  laser_flag = 1;
//				laser_TX = 0;
//			  tracking3.step =20;
//      }	
//		}			
//    break;	
		
		case 20:
		if(laser_TX == 1)   //13������������   ��ʼ9����
		{						
			if(tracking3.time <3.7f)   Walk_Ctrl(B, tracking3.speed); //�����  50cm    15*3.33=49.95 
			
			if(tracking3.time >3.7f && tracking3.time <3.90f)    Walk_Ctrl(S, tracking3.speed); 
			
			if(tracking3.time >3.9f &&  tracking3.time <4.7f)     Walk_Ctrl(R, LOW_SPEED); 
  				
			
			if(tracking3.time >4.7f) 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;			
		
		case 21:
		if(laser_TX == 1)   //9������������   ��ʼ7����
		{						
			if(tracking3.time <3.9f)   Walk_Ctrl(B, tracking3.speed); //�����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;	

		case 22:
		if(laser_TX == 1)   //7������������   ��ʼ6����
		{						
			if(tracking3.time <3.6f)   Walk_Ctrl(R, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;

		case 23:
		if(laser_TX == 1)   //6������������   ��ʼ5����
		{						
			if(tracking3.time <3.33f)   Walk_Ctrl(R, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking3.time >3.33f && tracking3.time <3.6f)  Walk_Ctrl(S, tracking3.speed);
			if(tracking3.time >3.6f &&  tracking3.time <4.8f)  Walk_Ctrl(B, LOW_SPEED);
			if(tracking3.time > 4.8f)
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;		

		case 24:
		if(laser_TX == 1)   //5������������   ��ʼ1����
		{						
			if(tracking3.time <3.1f)   Walk_Ctrl(B, tracking3.speed); //����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step =25;
//				my_flight.mode = Follow_Line_Mode;
//				contral_mode = 0;
			}
		}
		else tracking3.time = 0;
	 break;

//		case 51:    //��ֱ�� ���ȶ�һ��
//			
//	     cross_step = 2;

////	  if(dot.ok == 1)
////	  {
//			 if(tracking3.time < 6.0f) //Ĭ���ȶ�
//			 {
//					Walk_Ctrl(S, tracking3.speed); 
//			 }
//			 else
//			 {
//				my_flight.mode = Follow_Null;
//				contral_mode = 1;           //  ��ʼ����
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking3.time = 0;
//				tracking3.step = 25;	 
//			 }
////	  }
////	 else 
////	 {
////			tracking3.time -= dT;
////	 }		
//    break;	
//			 
		
		
		case 25:
		if(laser_TX == 1)   //1������������   ��ʼ2����
		{						
			if(tracking3.time <3.3f)   Walk_Ctrl(L,tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;	
		
		case 26:
		if(laser_TX == 1)   //2������������   ��ʼ3����
		{						
			if(tracking3.time <3.1f)   Walk_Ctrl(L, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;			
	
		case 27:
		if(laser_TX == 1)   //3������������   ��ʼ4����
		{						
			if(tracking3.time <3.0f)   Walk_Ctrl(L, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;		

		case 28:
		if(laser_TX == 1)   //4������������   ��ʼ8����
		{						
			if(tracking3.time <3.0f)   Walk_Ctrl(F, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;	
		
	

		case 29:
		if(laser_TX == 1)   //8������������   ��ʼ10����
		{						
			if(tracking3.time <3.0f)   Walk_Ctrl(F, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;	

		case 30:
		if(laser_TX == 1)   //10������������   ��ʼ14����
		{						
			if(tracking3.time <3.0f)   Walk_Ctrl(F, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;	


		case 31:
		if(laser_TX == 1)   //14������������   ��ʼ18����
		{						
			if(tracking3.time <3.0f)   Walk_Ctrl(F, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;	

		
		case 32:
		if(laser_TX == 1)   //18������������   ��ʼ19����
		{						
			if(tracking3.time <3.33f)   Walk_Ctrl(L, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;	

		
		case 33:
		if(laser_TX == 1)   //19������������   ��ʼ20����
		{						
			if(tracking3.time <3.33f)   Walk_Ctrl(L, tracking3.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking3.time >3.33f && tracking3.time <3.6f)  Walk_Ctrl(S,tracking3.speed);
			if(tracking3.time >3.6f  && tracking3.time <4.5f)  Walk_Ctrl(F,LOW_SPEED);
			if(tracking3.time >4.5f) 
			{
				Walk_Ctrl(S, tracking3.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
			}
		}
		else tracking3.time = 0;
	 break;	

		
	case 34:
		if(laser_TX == 1)   //20������������  ��ʼ����
		{						
			if(tracking3.time <5.0f)   Walk_Ctrl(L, tracking3.speed); //���ҷ�  50cm    15*4.0=60
			else 
			{ 
				laser_TX = 0;
				tracking3.time = 0;
				tracking3.step ++;
				my_flight.mode=Follow_Dot_Mode;
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking3.time = 0;
	 break;	
		
		
	 case 35:
		 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking3.speed);
			 
			  if(tracking3.time >4.0f)
				{
				   tracking3.step = 36;
					 tracking3.time = 0;      	
					 my_flight.mode=Follow_Null;
				   contral_mode = 1;
				}
		 }
		 
		 else 
		 {
			 //Walk_Ctrl(B,tracking3.speed);
			 tracking3.time = 0;
		 }
		 break;	

	 case 36:
	   if(tracking3.time <2.0f) Walk_Ctrl(L,tracking3.speed);
	   if(tracking3.time >2.0f && tracking3.time <15.5f)  Walk_Ctrl(B,tracking3.speed);
	   if(tracking3.time >=15.5f)
		 {
			  tracking3.step = 37;
			  tracking3.time = 0;
			 
			 	 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 dot_two_mode=1;
				 contral_mode = 0;	
		 }
		 break;
		 
	 case 37:
			 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking3.speed);
			 
			  if(tracking3.time >5.0f)
				{
					 User_Alt_ctrl =0;
				   tracking3.step = 99;
					 tracking3.time = 0; 
           dot_two_mode=0;					
				}
		 }
		 
		 else 
		{
				// tracking3.step = 99;
					 tracking3.time = 0; 
		 }	
		break;
		
		case 99:
			 tracking3.step += OneKey_Land();
		   tracking3.time =0;
			break;
				
		case 100:
			 if( tracking3.time >6.0f)
			 {
				  tracking3.step = 0;
				  tracking3.time =0;
				 FC_Lock();  //����
				 Pro_Ctr_step =0;
			 }
			 break;
  }
}

void Tracking_Ctrl4(float dT)    //  ������ ��� 17 16 14 18  ��ά�� 4  
{

	  tracking4.speed = TRACKING_SPEED; //�ٶȸ�ֵ
	
		if(Pro_Ctr_step != 4)
		{
			tracking4.time = 0;
			tracking4.step = 0;	
			return;
		}
		
		//��ʱ
		tracking4.time += dT;
		
	switch(tracking4.step)
	{
	     //�̿� 
		case 0:
			if( tracking4.time>=3)   //�������
			{							
		     FC_Unlock();
				 tracking4.time=0;
				 tracking4.step = 1; 
			}
			else
			{	
			     my_flight.mode = Follow_Null;
				   my_flight.color = NUL;
			}
		  break;	
			
		case 1:
			  if( tracking4.time >2.0f)		
				{
					 OneKey_Takeoff(145);			
					 tracking4.step = 2;
					 tracking4.time = 0;
				}
		  break;

		case 2:
			if( tracking4.time>=2.0f || ano_of.of_alt_cm>135 )   //�ȴ��ﵽ�߶�
			{	
				  tracking4.step = 3;
				  tracking4.time = 0;	
          contral_mode = 1;
  		}
		  break;
			
		case 3:
			 if(tracking4.time<9.5f)
			 {
			   Walk_Ctrl(F, tracking4.speed);//��ǰ�ɣ�Զ����ɵ�
			 }
			 
			 else 
			 {
				 tracking4.step = 4;   //���λ��
			   tracking4.time = 0; 			  
 				 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 contral_mode =0;
			 }
		  break;
			
		case 4:
			 if(dot.ok)
			{			
				Walk_Ctrl(S,tracking4.speed);	 //�ٶȽ���PID����
				if(tracking4.time >6.0f)       //2s�� Ӧ���ȶ�
				{
				  tracking4.time = 0;
				  tracking4.step = 5;
					contral_mode=1;  //����������    x,y ������� 				
					my_flight.mode  = Follow_Null;
		      my_flight.color = NUL;	
				} 
			}
			else	
			{
				Walk_Ctrl(F, tracking4.speed);//��ǰ Ѳ���
			  tracking4.time =0;
			}
		break;
			
			
		case 5:    //����������
		   if(tracking4.time <2.5f)		    Walk_Ctrl(R, tracking4.speed);
			 
		   if(tracking4.time == 2.5f)     Walk_Ctrl(S, tracking4.speed);
	
       if(tracking4.time >2.5f && tracking4.time <3.0f)   Walk_Ctrl(F, tracking4.speed);  
		
			 if(tracking4.time >=3.0f)
			 {
				  Walk_Ctrl(S, tracking4.speed);
			    tracking4.time  =0;
				  tracking4.step =6;
				  laser_flag=1;
			 }
			 break;

		case 6:
			if(laser_TX == 1)   //A������������  ��ʼ28����
			{			
				if(tracking4.time <2.9f)   Walk_Ctrl(F, tracking4.speed); //��ǰ��  50cm    15*3.33=49.95 
				if(tracking4.time >2.9f && tracking4.time < 3.1f)  Walk_Ctrl(S, tracking4.speed);   //��һ��
				if(tracking4.time >3.1f  && tracking4.time < 4.3f)  Walk_Ctrl(R,LOW_SPEED);   
				if(tracking4.time >4.3f) 
				{
					Walk_Ctrl(S, tracking4.speed);
				  laser_flag = 1;
					laser_TX = 0;
					tracking4.time = 0;
					tracking4.step = 7;
				}
			}
			else tracking4.time = 0;
			break;
			
		case 7:
			if(laser_TX == 1)   //28������������  ��ʼ27����
			{						
				if(tracking4.time <3.33f)   Walk_Ctrl(R, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking4.time >3.33f && tracking4.time <3.8f)  Walk_Ctrl(S, tracking4.speed);   //��һ��
				if(tracking4.time >3.8f  && tracking4.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking4.time >5.0f)
				{
					Walk_Ctrl(S, tracking4.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking4.time = 0;
					tracking4.step = 8;
				}
			}
			else tracking4.time = 0;
		 break;
		
		case 8:
			if(laser_TX == 1)   //27������������  ��ʼ26����
			{						
			  if(tracking4.time <3.33f)   Walk_Ctrl(R, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking4.time >3.33f && tracking4.time <3.6f)  Walk_Ctrl(S, tracking4.speed);   //��һ��
				if(tracking4.time >3.6f && tracking4.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking4.time >5.0f)
				{
					Walk_Ctrl(S, tracking4.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking4.time = 0;
					tracking4.step = 9;
				}
			}
			else tracking4.time = 0;
		 break;
			
			
//		case 9:
//			if(laser_TX == 1)   //27������������  ��ʼ26����
//			{						
//				if(tracking4.time <3.33f)   Walk_Ctrl(R, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
//				else 
//				{
//				  laser_flag = 1;
//					laser_TX = 0;
//					tracking4.time = 0;
//					tracking4.step = 10;
//				}
//			}
//			else tracking4.time = 0;
//		 break;			
		
		case 9:
			if(laser_TX == 1)   //26������������  ��ʼ25����
			{						
				if(tracking4.time <3.7f)   Walk_Ctrl(R, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking4.time >3.7f && tracking4.time <3.9f)  Walk_Ctrl(S, tracking4.speed);   //��һ��
				if(tracking4.time >3.9f && tracking4.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking4.time >5.0f)
				{
					Walk_Ctrl(S, tracking4.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking4.time = 0;
					tracking4.step = 10;
				}
			}
			else tracking4.time = 0;
		 break;	

		case 10:
			if(laser_TX == 1)   //25������������   ��ʼ24����
			{						
				if(tracking4.time <3.7f)   Walk_Ctrl(R, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking4.time >3.7f && tracking4.time <4.0f)  Walk_Ctrl(S, tracking4.speed);   //��һ��
				if(tracking4.time >4.0f  && tracking4.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking4.time >5.0f)
				{
					Walk_Ctrl(S, tracking4.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking4.time = 0;
					tracking4.step = 11;
				}
			}
			else tracking4.time = 0;
		 break;
			
//	  case 11:
//		if(laser_TX == 1)   //25������������   ��ʼ24����
//		{						
//			if(tracking4.time <3.33f)   Walk_Ctrl(R, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
//			else 
//			{
//				Walk_Ctrl(S, tracking4.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking4.time = 0;
//				tracking4.step = 12;
//			}
//		}
//		else tracking4.time = 0;
//	 break;
//		
		case 11:
		if(laser_TX == 1)   //24������������   17����  ����
		{						
			if(tracking4.time <3.33f)   Walk_Ctrl(B, tracking4.speed); //�����  50cm    15*3.33=49.95 
			if(tracking4.time >3.33f && tracking4.time <3.6f)  Walk_Ctrl(S, tracking4.speed);   //��һ��
			if(tracking4.time >3.6f  && tracking4.time <4.7f)  Walk_Ctrl(R, LOW_SPEED);   
			if(tracking4.time >4.7f) 
			{
				
				 Walk_Ctrl(S, tracking4.speed); 
				 if(tracking4.time >5.7f)
				 {
				   tracking4.time = 0;
				   tracking4.step = 13;				 
				 }
				//laser_flag = 1;
				//laser_TX = 0;
			}
		}
		else tracking4.time = 0;
	 break;
		
		case 13:
		if(laser_TX == 1)   //17������������  	16����
		{						
			if(tracking4.time <3.33f)   Walk_Ctrl(R, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
//			if(tracking4.time >3.33f&&  tracking4.time <4.3f)  Walk_Ctrl(S, tracking4.speed);
//			if(tracking4.time > 4.3f)
			else
			{
				Walk_Ctrl(S, tracking4.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step = 14;
			}
		}
		else tracking4.time = 0;
	 break;		

		case 14:
		if(laser_TX == 1)   //16������������   ��ʼ23����
		{						
			if(tracking4.time <3.3f)   Walk_Ctrl(F, tracking4.speed); //��ǰ��  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step = 15;
			}
		}
		else tracking4.time = 0;
	 break;		

		case 15:
		if(laser_TX == 1)   //23������������  
		{						
			if(tracking4.time <3.3f)   Walk_Ctrl(R, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S,tracking4.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step = 50;   
				my_flight.mode = Follow_Line_Mode;  //������ͷ��ֱ��
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking4.time = 0;
	 break;	
		
		case 50:    //��ֱ�� ���ȶ�һ��

		
					 if(tracking4.time <7.0f) //Ĭ���ȶ�
					 {
							Walk_Ctrl(S, 0); 
					 }
					 else
					 {
						my_flight.mode = Follow_Null;	   
						my_flight.color = NUL;
						contral_mode = 1;           //  ��ʼ����
						laser_flag = 1;
						laser_TX = 0;
						tracking4.time = 0;
						tracking4.step = 16;	 
					 }
					 
	
    break;	
			 
			 
		case 16:
		if(laser_TX == 1)   //22������������   ��ʼ15����
		{						
			if(tracking4.time <3.0f)   Walk_Ctrl(B, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;	
		
		case 17:
		if(laser_TX == 1)   //15������������   ��ʼ11����
		{						
			if(tracking4.time <3.0f)   Walk_Ctrl(B, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking4.time >3.0f && tracking4.time <3.4f)   Walk_Ctrl(S, tracking4.speed);
			if(tracking4.time >3.4f  && tracking4.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			if(tracking4.time >5.0f)
			{
				Walk_Ctrl(S, tracking4.speed);  
				  if(tracking4.time>6.0f)
					{
						laser_flag = 1;
						laser_TX = 0;
						tracking4.time = 0;
						tracking4.step ++;
					}
			}
		}
		else tracking4.time = 0;
	 break;
		
		case 18:
					if(laser_TX == 1)   //11������������   ��ʼ12����
		{	
			if(tracking4.time <3.33f)   Walk_Ctrl(L, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking4.time >3.33f &&tracking4.time <3.6f)   Walk_Ctrl(S, tracking4.speed);
			if(tracking4.time >3.6f  && tracking4.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			//if(tracking4.time >5.0f  && tracking4.time <6.0f)    Walk_Ctrl(F, LOW_SPEED);			
			if(tracking4.time >5.0f) 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;
		
		case 19:
		if(laser_TX == 1)   //12������������   ��ʼ13����
		{						
			if(tracking4.time <3.33f)   Walk_Ctrl(L, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking4.time >3.33f && tracking4.time <3.6f)     Walk_Ctrl(S, tracking4.speed);
			if(tracking4.time >3.6f  && tracking4.time <4.2f)     Walk_Ctrl(R, tracking4.speed);
			if(tracking4.time >4.2f  && tracking4 .time <5.7f)   Walk_Ctrl(F, LOW_SPEED);
			if(tracking4.time >5.7f) 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step =20;
			}
		}
		else tracking4.time = 0;
	 break;	
		
		
//		
//		case 52:    //��ֱ�� ���ȶ�һ��
//	  
//		  if(tracking4.time < 5.0f)
//			{
//			  Yaw_Dps_Out = 5;
//			}
//	  else 
//	  {
//			  tracking4.time = 0;
//			  laser_flag = 1;
//				laser_TX = 0;
//			  tracking4.step =20;
//      }	
//		}			
//    break;	
		
		case 20:
		if(laser_TX == 1)   //13������������   ��ʼ9����
		{						
			if(tracking4.time <3.7f)   Walk_Ctrl(B, tracking4.speed); //�����  50cm    15*3.33=49.95 
			
			if(tracking4.time >3.7f && tracking4.time <3.90f)    Walk_Ctrl(S, tracking4.speed); 
			
			if(tracking4.time >3.9f &&  tracking4.time <4.7f)     Walk_Ctrl(R, LOW_SPEED);
			
      if(tracking4.time >4.7f)			
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;			
		
		case 21:
		if(laser_TX == 1)   //9������������   ��ʼ7����
		{						
			if(tracking4.time <3.9f)   Walk_Ctrl(B, tracking4.speed); //�����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;	

		case 22:
		if(laser_TX == 1)   //7������������   ��ʼ6����
		{						
			if(tracking4.time <3.6f)   Walk_Ctrl(R, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;

		case 23:
		if(laser_TX == 1)   //6������������   ��ʼ5����
		{						
			if(tracking4.time <3.33f)   Walk_Ctrl(R, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking4.time >3.33f && tracking4.time <3.6f)  Walk_Ctrl(S, tracking4.speed);
			if(tracking4.time >3.6f &&  tracking4.time <4.8f)  Walk_Ctrl(B, LOW_SPEED);
			if(tracking4.time > 4.8f)
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;		

		case 24:
		if(laser_TX == 1)   //5������������   ��ʼ1����
		{						
			if(tracking4.time <3.1f)   Walk_Ctrl(B, tracking4.speed); //����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step =25;
//				my_flight.mode = Follow_Line_Mode;
//				contral_mode = 0;
			}
		}
		else tracking4.time = 0;
	 break;

//		case 51:    //��ֱ�� ���ȶ�һ��
//			
//	     cross_step = 2;

////	  if(dot.ok == 1)
////	  {
//			 if(tracking4.time < 6.0f) //Ĭ���ȶ�
//			 {
//					Walk_Ctrl(S, tracking4.speed); 
//			 }
//			 else
//			 {
//				my_flight.mode = Follow_Null;
//				contral_mode = 1;           //  ��ʼ����
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking4.time = 0;
//				tracking4.step = 25;	 
//			 }
////	  }
////	 else 
////	 {
////			tracking4.time -= dT;
////	 }		
//    break;	
//			 
		
		
		case 25:
		if(laser_TX == 1)   //1������������   ��ʼ2����
		{						
			if(tracking4.time <3.3f)   Walk_Ctrl(L,tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;	
		
		case 26:
		if(laser_TX == 1)   //2������������   ��ʼ3����
		{						
			if(tracking4.time <3.1f)   Walk_Ctrl(L, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;			
	
		case 27:
		if(laser_TX == 1)   //3������������   ��ʼ4����
		{						
			if(tracking4.time <3.0f)   Walk_Ctrl(L, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;		

		case 28:
		if(laser_TX == 1)   //4������������   ��ʼ8����
		{						
			if(tracking4.time <3.0f)   Walk_Ctrl(F, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;	
		
	

		case 29:
		if(laser_TX == 1)   //8������������   ��ʼ10����
		{						
			if(tracking4.time <3.0f)   Walk_Ctrl(F, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;	

		case 30:
		if(laser_TX == 1)   //10������������   ��14����
		{						
			if(tracking4.time <3.0f)   Walk_Ctrl(F, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;	


		case 31:
		if(laser_TX == 1)   //14������������  ��18����
		{						
			if(tracking4.time <3.0f)   Walk_Ctrl(F, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;	

		
		case 32:
		if(laser_TX == 1)   //18������������   ��ʼ19����
		{						
			if(tracking4.time <3.33f)   Walk_Ctrl(L, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;	

		
		case 33:
		if(laser_TX == 1)   //19������������   ��ʼ20����
		{						
			if(tracking4.time <3.33f)   Walk_Ctrl(L, tracking4.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking4.time >3.33f && tracking4.time <3.6f)  Walk_Ctrl(S,tracking4.speed);
			if(tracking4.time >3.6f  && tracking4.time <4.5f)  Walk_Ctrl(F,LOW_SPEED);
			if(tracking4.time >4.5f) 
			{
				Walk_Ctrl(S, tracking4.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
			}
		}
		else tracking4.time = 0;
	 break;	

		
	case 34:
		if(laser_TX == 1)   //20������������  ��ʼ����
		{						
			if(tracking4.time <5.0f)   Walk_Ctrl(L, tracking4.speed); //���ҷ�  50cm    15*4.0=60
			else 
			{ 
				laser_TX = 0;
				tracking4.time = 0;
				tracking4.step ++;
				my_flight.mode=Follow_Dot_Mode;
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking4.time = 0;
	 break;	
		
		
	 case 35:
		 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking4.speed);
			 
			  if(tracking4.time >4.0f)
				{
				   tracking4.step = 36;
					 tracking4.time = 0;      	
					 my_flight.mode=Follow_Null;
				   contral_mode = 1;
				}
		 }
		 
		 else 
		 {
			 //Walk_Ctrl(B,tracking4.speed);
			 tracking4.time = 0;
		 }
		 break;	

	 case 36:
	   if(tracking4.time <2.0f) Walk_Ctrl(L,tracking4.speed);
	   if(tracking4.time >2.0f && tracking4.time <15.5f)  Walk_Ctrl(B,tracking4.speed);
     if(tracking4.time  >=10.0f )   Car_Send(4);	  

	   if(tracking4.time >=15.5f)
		 {
			  tracking4.step = 37;
			  tracking4.time = 0;
			 
			 	 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 dot_two_mode=1;
				 contral_mode = 0;	
		 }
		 break;
		 
	 case 37:
			 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking4.speed);
			 
			  if(tracking4.time >3.0f)
				{
				   tracking4.step = 99;
					 tracking4.time = 0; 
           dot_two_mode=0;					
				}
		 }
		 
		 else 
		{
					 tracking4.step = 99;
					 tracking4.time = 0; 
		 }	
		break;
		
		case 99:
			 tracking4.step += OneKey_Land();
		   tracking4.time =0;
			break;
				
		case 100:
			 if( tracking4.time >4.0f)
			 {
				  tracking4.step = 0;
				  tracking4.time =0;
				 FC_Lock();  //����
				 Pro_Ctr_step =0;
			 }
			 break;
  }
}



void Tracking_Ctrl5(float dT)    //  ������ ��� 17 16 14 18  ��ά�� 5
{

	   tracking5.speed = TRACKING_SPEED; //�ٶȸ�ֵ
	
		if(Pro_Ctr_step != 5)
		{
			tracking5.time = 0;
			tracking5.step = 0;	
			return;
		}
		
		//��ʱ
		tracking5.time += dT;
		
	switch(tracking5.step)
	{
	     //�̿� 
		case 0:
			if( tracking5.time>=3)   //�������
			{							
		     FC_Unlock();
				 tracking5.time=0;
				 tracking5.step = 1; 
			}
			else
			{	
			     my_flight.mode = Follow_Null;
				   my_flight.color = NUL;
			}
		  break;	
			
		case 1:
			  if( tracking5.time >2.0f)		
				{
					 OneKey_Takeoff(145);			
					 tracking5.step = 2;
					 tracking5.time = 0;
				}
		  break;

		case 2:
			if( tracking5.time>=2.0f || ano_of.of_alt_cm>135 )   //�ȴ��ﵽ�߶�
			{	
				  tracking5.step = 3;
				  tracking5.time = 0;	
          contral_mode = 1;
  		}
		  break;
			
		case 3:
			 if(tracking5.time<9.5f)
			 {
			   Walk_Ctrl(F, tracking5.speed);//��ǰ�ɣ�Զ����ɵ�
			 }
			 
			 else 
			 {
				 tracking5.step = 4;   //���λ��
			   tracking5.time = 0; 			  
 				 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 contral_mode =0;
			 }
		  break;
			
		case 4:
			 if(dot.ok)
			{			
				Walk_Ctrl(S,tracking5.speed);	 //�ٶȽ���PID����
				if(tracking5.time >6.0f)       //2s�� Ӧ���ȶ�
				{
				  tracking5.time = 0;
				  tracking5.step = 5;
					contral_mode=1;  //����������    x,y ������� 				
					my_flight.mode  = Follow_Null;
		      my_flight.color = NUL;	
				} 
			}
			else	
			{
				Walk_Ctrl(F, tracking5.speed);//��ǰ Ѳ���
			  tracking5.time =0;
			}
		break;
			
			
		case 5:    //����������
		   if(tracking5.time <2.5f)		    Walk_Ctrl(R, tracking5.speed);
			 
		   if(tracking5.time == 2.5f)     Walk_Ctrl(S, tracking5.speed);
	
       if(tracking5.time >2.5f && tracking5.time <3.0f)   Walk_Ctrl(F, tracking5.speed);  
		
			 if(tracking5.time >=3.0f)
			 {
				  Walk_Ctrl(S, tracking5.speed);
			    tracking5.time  =0;
				  tracking5.step =6;
				  laser_flag=1;
			 }
			 break;

		case 6:
			if(laser_TX == 1)   //A������������  ��ʼ28����
			{			
				if(tracking5.time <2.9f)   Walk_Ctrl(F, tracking5.speed); //��ǰ��  50cm    15*3.33=49.95 
				if(tracking5.time >2.9f && tracking5.time < 3.1f)  Walk_Ctrl(S, tracking5.speed);   //��һ��
				if(tracking5.time >3.1f  && tracking5.time < 4.3f)  Walk_Ctrl(R,LOW_SPEED);   
				if(tracking5.time >4.3f) 
				{
					Walk_Ctrl(S, tracking5.speed);
				  laser_flag = 1;
					laser_TX = 0;
					tracking5.time = 0;
					tracking5.step = 7;
				}
			}
			else tracking5.time = 0;
			break;
			
		case 7:
			if(laser_TX == 1)   //28������������  ��ʼ27����
			{						
				if(tracking5.time <3.33f)   Walk_Ctrl(R, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking5.time >3.33f && tracking5.time <3.8f)  Walk_Ctrl(S, tracking5.speed);   //��һ��
				if(tracking5.time >3.8f  && tracking5.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking5.time >5.0f)
				{
					Walk_Ctrl(S, tracking5.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking5.time = 0;
					tracking5.step = 8;
				}
			}
			else tracking5.time = 0;
		 break;
		
		case 8:
			if(laser_TX == 1)   //27������������  ��ʼ26����
			{						
			  if(tracking5.time <3.33f)   Walk_Ctrl(R, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking5.time >3.33f && tracking5.time <3.6f)  Walk_Ctrl(S, tracking5.speed);   //��һ��
				if(tracking5.time >3.6f && tracking5.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking5.time >5.0f)
				{
					Walk_Ctrl(S, tracking5.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking5.time = 0;
					tracking5.step = 9;
				}
			}
			else tracking5.time = 0;
		 break;
			
			
//		case 9:
//			if(laser_TX == 1)   //27������������  ��ʼ26����
//			{						
//				if(tracking5.time <3.33f)   Walk_Ctrl(R, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
//				else 
//				{
//				  laser_flag = 1;
//					laser_TX = 0;
//					tracking5.time = 0;
//					tracking5.step = 10;
//				}
//			}
//			else tracking5.time = 0;
//		 break;			
		
		case 9:
			if(laser_TX == 1)   //26������������  ��ʼ25����
			{						
				if(tracking5.time <3.7f)   Walk_Ctrl(R, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking5.time >3.7f && tracking5.time <3.9f)  Walk_Ctrl(S, tracking5.speed);   //��һ��
				if(tracking5.time >3.9f && tracking5.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking5.time >5.0f)
				{
					Walk_Ctrl(S, tracking5.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking5.time = 0;
					tracking5.step = 10;
				}
			}
			else tracking5.time = 0;
		 break;	

		case 10:
			if(laser_TX == 1)   //25������������   ��ʼ24����
			{						
				if(tracking5.time <3.7f)   Walk_Ctrl(R, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking5.time >3.7f && tracking5.time <4.0f)  Walk_Ctrl(S, tracking5.speed);   //��һ��
				if(tracking5.time >4.0f  && tracking5.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking5.time >5.0f)
				{
					Walk_Ctrl(S, tracking5.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking5.time = 0;
					tracking5.step = 11;
				}
			}
			else tracking5.time = 0;
		 break;
			
//	  case 11:
//		if(laser_TX == 1)   //25������������   ��ʼ24����
//		{						
//			if(tracking5.time <3.33f)   Walk_Ctrl(R, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
//			else 
//			{
//				Walk_Ctrl(S, tracking5.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking5.time = 0;
//				tracking5.step = 12;
//			}
//		}
//		else tracking5.time = 0;
//	 break;
//		
		case 11:
		if(laser_TX == 1)   //24������������   17����  ����
		{						
			if(tracking5.time <3.33f)   Walk_Ctrl(B, tracking5.speed); //�����  50cm    15*3.33=49.95 
			if(tracking5.time >3.33f && tracking5.time <3.6f)  Walk_Ctrl(S, tracking5.speed);   //��һ��
			if(tracking5.time >3.6f  && tracking5.time <4.7f)  Walk_Ctrl(R, LOW_SPEED);   
			if(tracking5.time >4.7f) 
			{
				
				 Walk_Ctrl(S, tracking5.speed); 
				 if(tracking5.time >5.7f)
				 {
				   tracking5.time = 0;
				   tracking5.step = 13;				 
				 }
				//laser_flag = 1;
				//laser_TX = 0;
			}
		}
		else tracking5.time = 0;
	 break;
		
		case 13:
		if(laser_TX == 1)   //17������������  	16����
		{						
			if(tracking5.time <3.33f)   Walk_Ctrl(R, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
//			if(tracking5.time >3.33f&&  tracking5.time <4.3f)  Walk_Ctrl(S, tracking5.speed);
//			if(tracking5.time > 4.3f)
			else
			{
				Walk_Ctrl(S, tracking5.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step = 14;
			}
		}
		else tracking5.time = 0;
	 break;		

		case 14:
		if(laser_TX == 1)   //16������������   ��ʼ23����
		{						
			if(tracking5.time <3.3f)   Walk_Ctrl(F, tracking5.speed); //��ǰ��  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step = 15;
			}
		}
		else tracking5.time = 0;
	 break;		

		case 15:
		if(laser_TX == 1)   //23������������  
		{						
			if(tracking5.time <3.3f)   Walk_Ctrl(R, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S,tracking5.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step = 50;   
				my_flight.mode = Follow_Line_Mode;  //������ͷ��ֱ��
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking5.time = 0;
	 break;	
		
		case 50:    //��ֱ�� ���ȶ�һ��

		
					 if(tracking5.time <7.0f) //Ĭ���ȶ�
					 {
							Walk_Ctrl(S, 0); 
					 }
					 else
					 {
						my_flight.mode = Follow_Null;	   
						my_flight.color = NUL;
						contral_mode = 1;           //  ��ʼ����
						laser_flag = 1;
						laser_TX = 0;
						tracking5.time = 0;
						tracking5.step = 16;	 
					 }
					 
	
    break;	
			 
			 
		case 16:
		if(laser_TX == 1)   //22������������   ��ʼ15����
		{						
			if(tracking5.time <3.0f)   Walk_Ctrl(B, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;	
		
		case 17:
		if(laser_TX == 1)   //15������������   ��ʼ11����
		{						
			if(tracking5.time <3.0f)   Walk_Ctrl(B, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking5.time >3.0f && tracking5.time <3.4f)   Walk_Ctrl(S, tracking5.speed);
			if(tracking5.time >3.4f  && tracking5.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			if(tracking5.time >5.0f)
			{
				Walk_Ctrl(S, tracking5.speed);  
				  if(tracking5.time>6.0f)
					{
						laser_flag = 1;
						laser_TX = 0;
						tracking5.time = 0;
						tracking5.step ++;
					}
			}
		}
		else tracking5.time = 0;
	 break;
		
		case 18:
					if(laser_TX == 1)   //11������������   ��ʼ12����
		{	
			if(tracking5.time <3.33f)   Walk_Ctrl(L, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking5.time >3.33f &&tracking5.time <3.6f)   Walk_Ctrl(S, tracking5.speed);
			if(tracking5.time >3.6f  && tracking5.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			//if(tracking5.time >5.0f  && tracking5.time <6.0f)    Walk_Ctrl(F, LOW_SPEED);			
			if(tracking5.time >5.0f) 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;
		
		case 19:
		if(laser_TX == 1)   //12������������   ��ʼ13����
		{						
			if(tracking5.time <3.33f)   Walk_Ctrl(L, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking5.time >3.33f && tracking5.time <3.6f)     Walk_Ctrl(S, tracking5.speed);
			if(tracking5.time >3.6f  && tracking5.time <4.2f)     Walk_Ctrl(R, tracking5.speed);
			if(tracking5.time >4.2f  && tracking5 .time <5.7f)   Walk_Ctrl(F, LOW_SPEED);
			if(tracking5.time >5.7f) 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step =20;
			}
		}
		else tracking5.time = 0;
	 break;	
		
		
//		
//		case 52:    //��ֱ�� ���ȶ�һ��
//	  
//		  if(tracking5.time < 5.0f)
//			{
//			  Yaw_Dps_Out = 5;
//			}
//	  else 
//	  {
//			  tracking5.time = 0;
//			  laser_flag = 1;
//				laser_TX = 0;
//			  tracking5.step =20;
//      }	
//		}			
//    break;	
		
		case 20:
		if(laser_TX == 1)   //13������������   ��ʼ9����
		{						
			if(tracking5.time <3.7f)   Walk_Ctrl(B, tracking5.speed); //�����  50cm    15*3.33=49.95 
			
			if(tracking5.time >3.7f && tracking5.time <3.90f)    Walk_Ctrl(S, tracking5.speed); 
			
			if(tracking5.time >3.9f &&  tracking5.time <4.7f)     Walk_Ctrl(R, LOW_SPEED);
			
      if(tracking5.time >4.7f)			
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;			
		
		case 21:
		if(laser_TX == 1)   //9������������   ��ʼ7����
		{						
			if(tracking5.time <3.9f)   Walk_Ctrl(B, tracking5.speed); //�����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;	

		case 22:
		if(laser_TX == 1)   //7������������   ��ʼ6����
		{						
			if(tracking5.time <3.6f)   Walk_Ctrl(R, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;

		case 23:
		if(laser_TX == 1)   //6������������   ��ʼ5����
		{						
			if(tracking5.time <3.33f)   Walk_Ctrl(R, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking5.time >3.33f && tracking5.time <3.6f)  Walk_Ctrl(S, tracking5.speed);
			if(tracking5.time >3.6f &&  tracking5.time <4.8f)  Walk_Ctrl(B, LOW_SPEED);
			if(tracking5.time > 4.8f)
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;		

		case 24:
		if(laser_TX == 1)   //5������������   ��ʼ1����
		{						
			if(tracking5.time <3.1f)   Walk_Ctrl(B, tracking5.speed); //����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step =25;
//				my_flight.mode = Follow_Line_Mode;
//				contral_mode = 0;
			}
		}
		else tracking5.time = 0;
	 break;

//		case 51:    //��ֱ�� ���ȶ�һ��
//			
//	     cross_step = 2;

////	  if(dot.ok == 1)
////	  {
//			 if(tracking5.time < 6.0f) //Ĭ���ȶ�
//			 {
//					Walk_Ctrl(S, tracking5.speed); 
//			 }
//			 else
//			 {
//				my_flight.mode = Follow_Null;
//				contral_mode = 1;           //  ��ʼ����
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking5.time = 0;
//				tracking5.step = 25;	 
//			 }
////	  }
////	 else 
////	 {
////			tracking5.time -= dT;
////	 }		
//    break;	
//			 
		
		
		case 25:
		if(laser_TX == 1)   //1������������   ��ʼ2����
		{						
			if(tracking5.time <3.3f)   Walk_Ctrl(L,tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;	
		
		case 26:
		if(laser_TX == 1)   //2������������   ��ʼ3����
		{						
			if(tracking5.time <3.1f)   Walk_Ctrl(L, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;			
	
		case 27:
		if(laser_TX == 1)   //3������������   ��ʼ4����
		{						
			if(tracking5.time <3.0f)   Walk_Ctrl(L, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;		

		case 28:
		if(laser_TX == 1)   //4������������   ��ʼ8����
		{						
			if(tracking5.time <3.0f)   Walk_Ctrl(F, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;	
		
	

		case 29:
		if(laser_TX == 1)   //8������������   ��ʼ10����
		{						
			if(tracking5.time <3.0f)   Walk_Ctrl(F, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;	

		case 30:
		if(laser_TX == 1)   //10������������   ��14����
		{						
			if(tracking5.time <3.0f)   Walk_Ctrl(F, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;	


		case 31:
		if(laser_TX == 1)   //14������������  ��18����
		{						
			if(tracking5.time <3.0f)   Walk_Ctrl(F, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;	

		
		case 32:
		if(laser_TX == 1)   //18������������   ��ʼ19����
		{						
			if(tracking5.time <3.33f)   Walk_Ctrl(L, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;	

		
		case 33:
		if(laser_TX == 1)   //19������������   ��ʼ20����
		{						
			if(tracking5.time <3.33f)   Walk_Ctrl(L, tracking5.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking5.time >3.33f && tracking5.time <3.6f)  Walk_Ctrl(S,tracking5.speed);
			if(tracking5.time >3.6f  && tracking5.time <4.5f)  Walk_Ctrl(F,LOW_SPEED);
			if(tracking5.time >4.5f) 
			{
				Walk_Ctrl(S, tracking5.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
			}
		}
		else tracking5.time = 0;
	 break;	

		
	case 34:
		if(laser_TX == 1)   //20������������  ��ʼ����
		{						
			if(tracking5.time <5.0f)   Walk_Ctrl(L, tracking5.speed); //���ҷ�  50cm    15*4.0=60
			else 
			{ 
				laser_TX = 0;
				tracking5.time = 0;
				tracking5.step ++;
				my_flight.mode=Follow_Dot_Mode;
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking5.time = 0;
	 break;	
		
		
	 case 35:
		 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking5.speed);
			 
			  if(tracking5.time >4.0f)
				{
				   tracking5.step = 36;
					 tracking5.time = 0;      	
					 my_flight.mode=Follow_Null;
				   contral_mode = 1;
				}
		 }
		 
		 else 
		 {
			 //Walk_Ctrl(B,tracking5.speed);
			 tracking5.time = 0;
		 }
		 break;	

	 case 36:
	   if(tracking5.time <2.0f) Walk_Ctrl(L,tracking5.speed);
	   if(tracking5.time >2.0f && tracking5.time <15.5f)  Walk_Ctrl(B,tracking5.speed);
     if(tracking5.time  >=10.0f )   Car_Send(5);	  

	   if(tracking5.time >=15.5f)
		 {
			  tracking5.step = 37;
			  tracking5.time = 0;
			 
			 	 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 dot_two_mode=1;
				 contral_mode = 0;	
		 }
		 break;
		 
	 case 37:
			 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking5.speed);
			 
			  if(tracking5.time >3.0f)
				{
				   tracking5.step = 99;
					 tracking5.time = 0; 
           dot_two_mode=0;					
				}
		 }
		 
		 else 
		{
					 tracking5.step = 99;
					 tracking5.time = 0; 
		 }	
		break;
		
		case 99:
			 tracking5.step += OneKey_Land();
		   tracking5.time =0;
			break;
				
		case 100:
			 if( tracking5.time >4.0f)
			 {
				  tracking5.step = 0;
				  tracking5.time =0;
				 FC_Lock();  //����
				 Pro_Ctr_step =0;
			 }
			 break;
  }
}


void Tracking_Ctrl6(float dT)    //  ������ ���  17 16 14 18 ��ά�� 3
{
	  tracking6.speed = TRACKING_SPEED; //�ٶȸ�ֵ
	
		if(Pro_Ctr_step != 6)
		{
			tracking6.time = 0;
			tracking6.step = 0;	
			return;
		}
		
		//��ʱ
		tracking6.time += dT;
		
	switch(tracking6.step)
	{
	     //�̿� 
		case 0:
			if( tracking6.time>=3)   //�������
			{							
		     FC_Unlock();
				 tracking6.time=0;
				 tracking6.step = 1; 
			}
			else
			{	
			     my_flight.mode = Follow_Null;
				   my_flight.color = NUL;
			}
		  break;	
			
		case 1:
			  if( tracking6.time >2.0f)		
				{
					 OneKey_Takeoff(145);			
					 tracking6.step = 2;
					 tracking6.time = 0;
				}
		  break;

		case 2:
			if( tracking6.time>=2.0f || ano_of.of_alt_cm>135 )   //�ȴ��ﵽ�߶�
			{	
				  tracking6.step = 3;
				  tracking6.time = 0;	
          contral_mode = 1;
  		}
		  break;
			
		case 3:
			 if(tracking6.time<9.5f)
			 {
			   Walk_Ctrl(F, tracking6.speed);//��ǰ�ɣ�Զ����ɵ�
			 }
			 
			 else 
			 {
				 tracking6.step = 4;   //���λ��
			   tracking6.time = 0; 			  
 				 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 contral_mode =0;
			 }
		  break;
			
		case 4:
			 if(dot.ok)
			{			
				Walk_Ctrl(S,tracking6.speed);	 //�ٶȽ���PID����
				if(tracking6.time >6.0f)       //2s�� Ӧ���ȶ�
				{
				  tracking6.time = 0;
				  tracking6.step = 5;
					contral_mode=1;  //����������    x,y ������� 				
					my_flight.mode  = Follow_Null;
		      my_flight.color = NUL;	
				} 
			}
			else	
			{
				Walk_Ctrl(F, tracking6.speed);//��ǰ Ѳ���
			  tracking6.time =0;
			}
		break;
			
			
		case 5:    //����������
		   if(tracking6.time <2.5f)		    Walk_Ctrl(R, tracking6.speed);
			 
		   if(tracking6.time == 2.5f)     Walk_Ctrl(S, tracking6.speed);
	
       if(tracking6.time >2.5f && tracking6.time <3.0f)   Walk_Ctrl(F, tracking6.speed);  
		
			 if(tracking6.time >=3.0f)
			 {
				  Walk_Ctrl(S, tracking6.speed);
			    tracking6.time  =0;
				  tracking6.step =6;
				  laser_flag=1;
			 }
			 break;

		case 6:
			if(laser_TX == 1)   //A������������  ��ʼ28����
			{			
				if(tracking6.time <2.9f)   Walk_Ctrl(F, tracking6.speed); //��ǰ��  50cm    15*3.33=49.95 
				if(tracking6.time >2.9f && tracking6.time < 3.1f)  Walk_Ctrl(S, tracking6.speed);   //��һ��
				if(tracking6.time >3.1f  && tracking6.time < 4.3f)  Walk_Ctrl(R,LOW_SPEED);   
				if(tracking6.time >4.3f) 
				{
					Walk_Ctrl(S, tracking6.speed);
				  laser_flag = 1;
					laser_TX = 0;
					tracking6.time = 0;
					tracking6.step = 7;
				}
			}
			else tracking6.time = 0;
			break;
			
		case 7:
			if(laser_TX == 1)   //28������������  ��ʼ27����
			{						
				if(tracking6.time <3.33f)   Walk_Ctrl(R, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking6.time >3.33f && tracking6.time <3.8f)  Walk_Ctrl(S, tracking6.speed);   //��һ��
				if(tracking6.time >3.8f  && tracking6.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking6.time >5.0f)
				{
					Walk_Ctrl(S, tracking6.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking6.time = 0;
					tracking6.step = 8;
				}
			}
			else tracking6.time = 0;
		 break;
		
		case 8:
			if(laser_TX == 1)   //27������������  ��ʼ26����
			{						
			  if(tracking6.time <3.33f)   Walk_Ctrl(R, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking6.time >3.33f && tracking6.time <3.6f)  Walk_Ctrl(S, tracking6.speed);   //��һ��
				if(tracking6.time >3.6f && tracking6.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking6.time >5.0f)
				{
					Walk_Ctrl(S, tracking6.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking6.time = 0;
					tracking6.step = 9;
				}
			}
			else tracking6.time = 0;
		 break;
			
			
//		case 9:
//			if(laser_TX == 1)   //27������������  ��ʼ26����
//			{						
//				if(tracking6.time <3.33f)   Walk_Ctrl(R, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
//				else 
//				{
//				  laser_flag = 1;
//					laser_TX = 0;
//					tracking6.time = 0;
//					tracking6.step = 10;
//				}
//			}
//			else tracking6.time = 0;
//		 break;			
		
		case 9:
			if(laser_TX == 1)   //26������������  ��ʼ25����
			{						
				if(tracking6.time <3.7f)   Walk_Ctrl(R, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking6.time >3.7f && tracking6.time <3.9f)  Walk_Ctrl(S, tracking6.speed);   //��һ��
				if(tracking6.time >3.9f && tracking6.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking6.time >5.0f)
				{
					Walk_Ctrl(S, tracking6.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking6.time = 0;
					tracking6.step = 10;
				}
			}
			else tracking6.time = 0;
		 break;	

		case 10:
			if(laser_TX == 1)   //25������������   ��ʼ24����
			{						
				if(tracking6.time <3.7f)   Walk_Ctrl(R, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
				if(tracking6.time >3.7f && tracking6.time <4.0f)  Walk_Ctrl(S, tracking6.speed);   //��һ��
				if(tracking6.time >4.0f  && tracking6.time < 5.0f)  Walk_Ctrl(B, LOW_SPEED);   //����һ��
				if(tracking6.time >5.0f)
				{
					Walk_Ctrl(S, tracking6.speed); 
				  laser_flag = 1;
					laser_TX = 0;
					tracking6.time = 0;
					tracking6.step = 11;
				}
			}
			else tracking6.time = 0;
		 break;
			
//	  case 11:
//		if(laser_TX == 1)   //25������������   ��ʼ24����
//		{						
//			if(tracking6.time <3.33f)   Walk_Ctrl(R, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
//			else 
//			{
//				Walk_Ctrl(S, tracking6.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking6.time = 0;
//				tracking6.step = 12;
//			}
//		}
//		else tracking6.time = 0;
//	 break;
//		
		case 11:
		if(laser_TX == 1)   //24������������   17����  ����
		{						
			if(tracking6.time <3.33f)   Walk_Ctrl(B, tracking6.speed); //�����  50cm    15*3.33=49.95 
			if(tracking6.time >3.33f && tracking6.time <3.6f)  Walk_Ctrl(S, tracking6.speed);   //��һ��
			if(tracking6.time >3.6f  && tracking6.time <4.7f)  Walk_Ctrl(R, LOW_SPEED);   
			if(tracking6.time >4.7f) 
			{
				
				 Walk_Ctrl(S, tracking6.speed); 
				 if(tracking6.time >5.7f)
				 {
				   tracking6.time = 0;
				   tracking6.step = 13;				 
				 }
				//laser_flag = 1;
				//laser_TX = 0;
			}
		}
		else tracking6.time = 0;
	 break;
		
		case 13:
		if(laser_TX == 1)   //17������������  	16����
		{						
			if(tracking6.time <3.33f)   Walk_Ctrl(R, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
//			if(tracking6.time >3.33f&&  tracking6.time <4.3f)  Walk_Ctrl(S, tracking6.speed);
//			if(tracking6.time > 4.3f)
			else
			{
				Walk_Ctrl(S, tracking6.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step = 14;
			}
		}
		else tracking6.time = 0;
	 break;		

		case 14:
		if(laser_TX == 1)   //16������������   ��ʼ23����
		{						
			if(tracking6.time <3.3f)   Walk_Ctrl(F, tracking6.speed); //��ǰ��  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step = 15;
			}
		}
		else tracking6.time = 0;
	 break;		

		case 15:
		if(laser_TX == 1)   //23������������  
		{						
			if(tracking6.time <3.3f)   Walk_Ctrl(R, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S,tracking6.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step = 50;   
				my_flight.mode = Follow_Line_Mode;  //������ͷ��ֱ��
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking6.time = 0;
	 break;	
		
		case 50:    //��ֱ�� ���ȶ�һ��

		
					 if(tracking6.time <7.0f) //Ĭ���ȶ�
					 {
							Walk_Ctrl(S, 0); 
					 }
					 else
					 {
						my_flight.mode = Follow_Null;	   
						my_flight.color = NUL;
						contral_mode = 1;           //  ��ʼ����
						laser_flag = 1;
						laser_TX = 0;
						tracking6.time = 0;
						tracking6.step = 16;	 
					 }
					 
	
    break;	
			 
			 
		case 16:
		if(laser_TX == 1)   //22������������   ��ʼ15����
		{						
			if(tracking6.time <3.0f)   Walk_Ctrl(B, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;	
		
		case 17:
		if(laser_TX == 1)   //15������������   ��ʼ11����
		{						
			if(tracking6.time <3.0f)   Walk_Ctrl(B, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking6.time >3.0f && tracking6.time <3.4f)   Walk_Ctrl(S, tracking6.speed);
			if(tracking6.time >3.4f  && tracking6.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			if(tracking6.time >5.0f)
			{
				Walk_Ctrl(S, tracking6.speed);  
				  if(tracking6.time>6.0f)
					{
						laser_flag = 1;
						laser_TX = 0;
						tracking6.time = 0;
						tracking6.step ++;
					}
			}
		}
		else tracking6.time = 0;
	 break;
		
		case 18:
					if(laser_TX == 1)   //11������������   ��ʼ12����
		{	
			if(tracking6.time <3.33f)   Walk_Ctrl(L, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking6.time >3.33f &&tracking6.time <3.6f)   Walk_Ctrl(S, tracking6.speed);
			if(tracking6.time >3.6f  && tracking6.time <5.0f)    Walk_Ctrl(R, LOW_SPEED);
			//if(tracking6.time >5.0f  && tracking6.time <6.0f)    Walk_Ctrl(F, LOW_SPEED);			
			if(tracking6.time >5.0f) 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;
		
		case 19:
		if(laser_TX == 1)   //12������������   ��ʼ13����
		{						
			if(tracking6.time <3.33f)   Walk_Ctrl(L, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking6.time >3.33f && tracking6.time <3.6f)     Walk_Ctrl(S, tracking6.speed);
			if(tracking6.time >3.6f  && tracking6.time <4.2f)     Walk_Ctrl(R, tracking6.speed);
			if(tracking6.time >4.2f  && tracking6 .time <5.7f)   Walk_Ctrl(F, LOW_SPEED);
			if(tracking6.time >5.7f) 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step =20;
			}
		}
		else tracking6.time = 0;
	 break;	
		
		
//		
//		case 52:    //��ֱ�� ���ȶ�һ��
//	  
//		  if(tracking6.time < 5.0f)
//			{
//			  Yaw_Dps_Out = 5;
//			}
//	  else 
//	  {
//			  tracking6.time = 0;
//			  laser_flag = 1;
//				laser_TX = 0;
//			  tracking6.step =20;
//      }	
//		}			
//    break;	
		
		case 20:
		if(laser_TX == 1)   //13������������   ��ʼ9����
		{						
			if(tracking6.time <3.7f)   Walk_Ctrl(B, tracking6.speed); //�����  50cm    15*3.33=49.95 
			
			if(tracking6.time >3.7f && tracking6.time <3.90f)    Walk_Ctrl(S, tracking6.speed); 
			
			if(tracking6.time >3.9f &&  tracking6.time <4.7f)     Walk_Ctrl(R, LOW_SPEED);
			
      if(tracking6.time >4.7f)			
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;			
		
		case 21:
		if(laser_TX == 1)   //9������������   ��ʼ7����
		{						
			if(tracking6.time <3.9f)   Walk_Ctrl(B, tracking6.speed); //�����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;	

		case 22:
		if(laser_TX == 1)   //7������������   ��ʼ6����
		{						
			if(tracking6.time <3.6f)   Walk_Ctrl(R, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;

		case 23:
		if(laser_TX == 1)   //6������������   ��ʼ5����
		{						
			if(tracking6.time <3.33f)   Walk_Ctrl(R, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking6.time >3.33f && tracking6.time <3.6f)  Walk_Ctrl(S, tracking6.speed);
			if(tracking6.time >3.6f &&  tracking6.time <4.8f)  Walk_Ctrl(B, LOW_SPEED);
			if(tracking6.time > 4.8f)
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;		

		case 24:
		if(laser_TX == 1)   //5������������   ��ʼ1����
		{						
			if(tracking6.time <3.1f)   Walk_Ctrl(B, tracking6.speed); //����  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step =25;
//				my_flight.mode = Follow_Line_Mode;
//				contral_mode = 0;
			}
		}
		else tracking6.time = 0;
	 break;

//		case 51:    //��ֱ�� ���ȶ�һ��
//			
//	     cross_step = 2;

////	  if(dot.ok == 1)
////	  {
//			 if(tracking6.time < 6.0f) //Ĭ���ȶ�
//			 {
//					Walk_Ctrl(S, tracking6.speed); 
//			 }
//			 else
//			 {
//				my_flight.mode = Follow_Null;
//				contral_mode = 1;           //  ��ʼ����
//				laser_flag = 1;
//				laser_TX = 0;
//				tracking6.time = 0;
//				tracking6.step = 25;	 
//			 }
////	  }
////	 else 
////	 {
////			tracking6.time -= dT;
////	 }		
//    break;	
//			 
		
		
		case 25:
		if(laser_TX == 1)   //1������������   ��ʼ2����
		{						
			if(tracking6.time <3.3f)   Walk_Ctrl(L,tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;	
		
		case 26:
		if(laser_TX == 1)   //2������������   ��ʼ3����
		{						
			if(tracking6.time <3.1f)   Walk_Ctrl(L, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;			
	
		case 27:
		if(laser_TX == 1)   //3������������   ��ʼ4����
		{						
			if(tracking6.time <3.0f)   Walk_Ctrl(L, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;		

		case 28:
		if(laser_TX == 1)   //4������������   ��ʼ8����
		{						
			if(tracking6.time <3.0f)   Walk_Ctrl(F, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;	
		
	

		case 29:
		if(laser_TX == 1)   //8������������   ��ʼ10����
		{						
			if(tracking6.time <3.0f)   Walk_Ctrl(F, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;	

		case 30:
		if(laser_TX == 1)   //10������������   ��14����
		{						
			if(tracking6.time <3.0f)   Walk_Ctrl(F, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;	


		case 31:
		if(laser_TX == 1)   //14������������  ��18����
		{						
			if(tracking6.time <3.0f)   Walk_Ctrl(F, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
//				laser_flag = 1;
//				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;	

		
		case 32:
		if(laser_TX == 1)   //18������������   ��ʼ19����
		{						
			if(tracking6.time <3.33f)   Walk_Ctrl(L, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			else 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;	

		
		case 33:
		if(laser_TX == 1)   //19������������   ��ʼ20����
		{						
			if(tracking6.time <3.33f)   Walk_Ctrl(L, tracking6.speed); //���ҷ�  50cm    15*3.33=49.95 
			if(tracking6.time >3.33f && tracking6.time <3.6f)  Walk_Ctrl(S,tracking6.speed);
			if(tracking6.time >3.6f  && tracking6.time <4.5f)  Walk_Ctrl(F,LOW_SPEED);
			if(tracking6.time >4.5f) 
			{
				Walk_Ctrl(S, tracking6.speed); 
				laser_flag = 1;
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
			}
		}
		else tracking6.time = 0;
	 break;	

		
	case 34:
		if(laser_TX == 1)   //20������������  ��ʼ����
		{						
			if(tracking6.time <5.0f)   Walk_Ctrl(L, tracking6.speed); //���ҷ�  50cm    15*4.0=60
			else 
			{ 
				laser_TX = 0;
				tracking6.time = 0;
				tracking6.step ++;
				my_flight.mode=Follow_Dot_Mode;
				my_flight.color = RED;
				contral_mode = 0;
			}
		}
		else tracking6.time = 0;
	 break;	
		
		
	 case 35:
		 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking6.speed);
			 
			  if(tracking6.time >4.0f)
				{
				   tracking6.step = 36;
					 tracking6.time = 0;      	
					 my_flight.mode=Follow_Null;
				   contral_mode = 1;
				}
		 }
		 
		 else 
		 {
			 //Walk_Ctrl(B,tracking6.speed);
			 tracking6.time = 0;
		 }
		 break;	

	 case 36:
	   if(tracking6.time <2.0f) Walk_Ctrl(L,tracking6.speed);
	   if(tracking6.time >2.0f && tracking6.time <15.5f)  Walk_Ctrl(B,tracking6.speed);
     if(tracking6.time  >=10.0f )   Car_Send(3);	  

	   if(tracking6.time >=12.5f)
		 {
			  tracking6.step = 37;
			  tracking6.time = 0;
			 
			 	 my_flight.mode = Follow_Dot_Mode;
		     my_flight.color = RED;
				 dot_two_mode=1;
				 contral_mode = 0;	
		 }
		 break;
		 
	 case 37:
			 if(dot.ok)
		 {
		    Walk_Ctrl(S,tracking6.speed);
			 
			  if(tracking6.time >3.0f)
				{
				   tracking6.step = 99;
					 tracking6.time = 0; 
           dot_two_mode=0;					
				}
		 }
		 
		 else 
		{
					 tracking6.step = 99;
					 tracking6.time = 0; 
		 }	
		break;
		
		case 99:
			 tracking6.step += OneKey_Land();
		   tracking6.time =0;
			break;
				
		case 100:
			 if( tracking6.time >3.0f)
			 {
				  tracking6.step = 0;
				  tracking6.time =0;
				 FC_Lock();  //����
				 Pro_Ctr_step =0;
			 }
			 break;
  }
}
