#include "Ano_FlightCtrl.h"
#include "Ano_Pid.h"
#include "Drv_Yaw.h"
#include "Ano_LocCtrl.h"
#include "Drv_mv.h"
#include "Drv_mv1.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "Ano_Alt.h"
#include "Drv_bluetooch.h"
#include "LX_FC_State.h"

void ALL_PID_Init(void)
{
	
   User_my_yaw_PID_Init(); //巡线修正Yaw PID
		
   User_Alt_PID_Init();  //高度环PID
	
   User_line_PID_Init(); //巡线

	 User_Len_PID_Init();   //绕杆定点距离
	
   User_Loc_PID_Init();   //下openmv追小车
}

u8 Pro_Ctr_step1=0;
u8 Pro_Ctr_step2=0;
extern u8 Pro_Ctr_step;
extern _fc_state_st fc_sta;
u8 mode ;
struct MYMODE my_flight;
struct MYMODE my_flight_mv1;
extern u8 sum_sum;

void Flight_Mode_Set(u32 dT_us)  //遥控器通道 && openmv模式切换
{
	
	    //用静态变量记录一键起飞/降落指令已经执行。
	  static u8   one_key_take = 1,two_key_take=1,three_key_take=1;
    //判断有遥控信号才执行
    if (rc_in.fail_safe == 0)
    {
				//判断第6通道拨杆位置 800<CH_6<1200    //第二档
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {
             //还没有执行
            if (one_key_take == 0)
            {
                //标记已经执行
                Pro_Ctr_step =1;
							  one_key_take=1;
            }
						
        }
        else
        {
               one_key_take=0;
        }	
    }
		
		    //判断有遥控信号才执行
    if (rc_in.fail_safe == 0)
    {
				//判断第6通道拨杆位置 800<CH_6<1200    //第二档
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
             //还没有执行
            if (three_key_take == 0)
            {
                //标记已经执行
                Pro_Ctr_step =2;
							  //OneKey_Land();
							  three_key_take=1;
            }
						
        }
        else
        {
               three_key_take=0;
        }	
    }
		
		    //判断有遥控信号才执行
    if (rc_in.fail_safe == 0)
    {
				//判断第6通道拨杆位置 800<CH_6<1200    //第二档
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
             //还没有执行
            if (two_key_take == 0)
            {
                //标记已经执行
                Pro_Ctr_step =0;
							  two_key_take=1;
            }
						
        }
        else
        {
               two_key_take=0;
        }	
    }		
		
//	//按键切换模式
//	if(Pro_Ctr_step == 10)
//	{
//		mode++;
//		Pro_Ctr_step =0;
//		if(mode ==4)  mode =1;
//		if(mode ==1)  LX_Change_Mode(2);
//		if(mode ==2)  LX_Change_Mode(1);
//		if(mode ==3)  LX_Change_Mode(3);
//	}
		
		if(my_flight.mode_old != my_flight.mode || my_flight.color_old != my_flight.color)   //前摄像头  Uart3
	  {
			my_flight.mode_old = my_flight.mode;
			my_flight.color_old = my_flight.color;
			switch(my_flight.mode)
			{
				case Follow_Dot_Mode : 
														switch(my_flight.color)
														{
															case RED: Player_Send_Set(CHECK_DOT,RED); break; 
															case BLK: Player_Send_Set(CHECK_DOT,BLK); break;
															case GRN: Player_Send_Set(CHECK_DOT,GRN); break;
															case YAL: Player_Send_Set(CHECK_DOT,YAL); break;															
															default :Player_Send_Set(CHECK_NULL,NUL); break;
														}
														break;
				case Follow_Line_Mode:
														switch(my_flight.color)
														{
															case RED:Player_Send_Set(CHECK_LINE,RED); break; 
															case BLK:Player_Send_Set(CHECK_LINE,BLK); break;
															case GRN:Player_Send_Set(CHECK_LINE,GRN); break;
															default :Player_Send_Set(CHECK_NULL,NUL); break;
														}	
														break;													
				case Follow_RGDot_Mode:
														switch(my_flight.color)
														{
															case RED:Player_Send_Set(CHECK_LINE_DOT,RED); break; 
															case BLK:Player_Send_Set(CHECK_LINE_DOT,BLK); break;
															case GRN:Player_Send_Set(CHECK_LINE_DOT,GRN); break;
														  case YAL: Player_Send_Set(CHECK_LINE_DOT,YAL); break;	
															default :Player_Send_Set(CHECK_NULL,NUL); break;
														}	
														break;
				default :          Player_Send_Set(CHECK_NULL,NUL);             break;
		  }										
	 }

	 
		if(my_flight_mv1.mode_old != my_flight_mv1.mode || my_flight_mv1.color_old != my_flight_mv1.color)   //下摄像头 Uart1
	  {
			my_flight_mv1.mode_old = my_flight_mv1.mode;
			my_flight_mv1.color_old = my_flight_mv1.color;
			switch(my_flight_mv1.mode)
			{
				case Follow_Dot_Mode : 
														switch(my_flight_mv1.color)
														{
															case RED: Player_Sendmv1_Set(CHECK_DOT,RED); break; 
															case BLK: Player_Sendmv1_Set(CHECK_DOT,BLK); break;
															case GRN: Player_Sendmv1_Set(CHECK_DOT,GRN); break;
															case YAL: Player_Sendmv1_Set(CHECK_DOT,YAL); break;															
															default :Player_Sendmv1_Set(CHECK_NULL,NUL); break;
														}
														break;
				case Follow_Line_Mode:
														switch(my_flight_mv1.color)
														{
															case RED:Player_Sendmv1_Set(CHECK_LINE,RED); break; 
															case BLK:Player_Sendmv1_Set(CHECK_LINE,BLK); break;
															case GRN:Player_Sendmv1_Set(CHECK_LINE,GRN); break;
															default :Player_Sendmv1_Set(CHECK_NULL,NUL); break;
														}	
														break;													
				case Follow_RGDot_Mode:
														switch(my_flight_mv1.color)
														{
															case RED:Player_Sendmv1_Set(CHECK_LINE_DOT,RED); break; 
															case BLK:Player_Sendmv1_Set(CHECK_LINE_DOT,BLK); break;
															case GRN:Player_Sendmv1_Set(CHECK_LINE_DOT,GRN); break;
														  case YAL: Player_Sendmv1_Set(CHECK_LINE_DOT,YAL); break;	
															default :Player_Sendmv1_Set(CHECK_NULL,NUL); break;
														}	
														break;
				default :          Player_Sendmv1_Set(CHECK_NULL,NUL);             break;
		  }										
	 }
		
	  //c8t6传输数据
		Send_Set_mode(Pro_Ctr_step,fc_sta.fc_mode_sta);
	  Car_Send_Set(sum_sum);
}

