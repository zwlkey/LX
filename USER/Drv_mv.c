#include "Drv_mv.h"
#include "Ano_Math.h"
#include "math.h"
#include "Drv_Uart.h"
#include "Ano_FlightCtrl.h"
#include "Drv_AnoOf.h"
#include "ANO_LX.h"

struct _line_check_ line;
struct _dot_check_ dot;

struct _line_check_ line_mv1;
struct _dot_check_ dot_mv1;
struct _MV_ 		MV_mv1;


float Mv_loc_x,Mv_loc_y;

extern _ano_of_st ano_of;  //光流输出数据
extern _fc_att_un fc_att;  //姿态角_欧拉角格式
struct _rol_pit_ mcu_rx; 

u8 Set_Mv_Mode[50];
u8 Set_Angle_Mode[50];


//视觉定位设置
void Player_Send_Set(u8 mode,u8 color)
{
	u8 _cnt=0;
	
	Set_Mv_Mode[_cnt++]=0xAA;
	Set_Mv_Mode[_cnt++]=0xAF;
	Set_Mv_Mode[_cnt++]=0xF1;
	Set_Mv_Mode[_cnt++]=0;
	
	Set_Mv_Mode[_cnt++] = mode;
  Set_Mv_Mode[_cnt++] = color;
	Set_Mv_Mode[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += Set_Mv_Mode[i];
	
	Set_Mv_Mode[_cnt++] = sum;
	DrvUart3SendBuf(Set_Mv_Mode, _cnt);
}

//Openmv数据接收
void Player_Mv_Receive(u8 data)
{
	static u8 RxBuffer[32];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	switch(state)
	{
		case 0:
			if(data==0xAA)
			{
				state=1;
				RxBuffer[0]=data;
			}else state = 0;
		break;
			
		case 1:
			if(data==0xAA)
			{
				state=2;
				RxBuffer[1]=data;
			}else state = 0;
		break;
			
		case 2:
			if(data<0xFF)
			{
				state=3;
				RxBuffer[2]=data;
			}else state = 0;
		break;
			
		case 3:
			if(data<33)
			{
				state = 4;
				RxBuffer[3]=data;
				_data_len = data;
				_data_cnt = 0;
			}else state = 0;
		break;

		case 4:
			if(_data_len>0)
			{
				_data_len--;
				RxBuffer[4+_data_cnt++]=data;
				if(_data_len==0)	state = 5;
			}else state = 0;
		break;			
			
		case 5:
			state = 0;
			RxBuffer[4+_data_cnt]=data;
			Player_Duty(RxBuffer);
		break;	

		default:	state = 0;	break;
	}
}

struct _MV_ 		MV;
extern struct MYMODE my_flight;

//视觉标志位检测
void MV_Flag_Check(u8 flag0,u8 flag1)
{
	static u8 cnt3,cnt4;
	static u8 cnt3_,cnt4_;
	
	//角度检测标志位
	if( flag0 ){cnt3++;cnt3_=0; if(cnt3>200)cnt3=200;} else {cnt3=0;cnt3_++;if(cnt3_>200)cnt3_=200;}
	if(cnt3>5) line.angle_ok = 1; else if(cnt3_>10) line.angle_ok  = 0;//判断角度是否OK
	
	//点检测标志位
	if( flag1 ){cnt4++;cnt4_=0; if(cnt4>200)cnt4=200;} else {cnt4=0;cnt4_++;if(cnt4_>200)cnt4_=200;}
	if(cnt4>20) dot.ok = 1; else if(cnt4_>20) dot.ok = 0;//判断是否检测到点
	
}

extern u8 dot_free_flag;
//视觉定位数据解析
//视觉定位数据解析
void Player_Duty(u8 *data_buf)
{
	switch( *(data_buf+2) )
	{
		case 0xF2://点检测数据
				dot.x = ((s16)(*(data_buf+4)<<8)|*(data_buf+5));
				dot.y = ((s16)(*(data_buf+6)<<8)|*(data_buf+7));
				dot.flag = *(data_buf+8);	
		break;
		
		case 0xF3://线检测数据	//线倾角
//				line.angle_y = ((s16)(*(data_buf+4)<<8)|*(data_buf+5)); line.angle_y *= 0.01f;
//		    line.angle_x = ((s16)(*(data_buf+6)<<8)|*(data_buf+5)); line.angle_y *= 0.01f;
//				line.flag    = *(data_buf+6);
		break;	
	}
}


void MV_Decoupling(u8 dT_ms)         //下openmv旋转解耦
{
	float distance_l,distance_w,high_temp;
	
	//计算每个像素代表的实际距离
	high_temp = ano_of.of_alt_cm * 10.0f;//高度单位转换为mm
	
	distance_l = (Pictur_L / 160.0f) * ( LIMIT(high_temp,100,2000) / Camera_H )*0.1f;//计算每个像素代表的实际距离,单位cm;
	distance_w = (Pictur_W / 120.0f) * ( LIMIT(high_temp,100,2000) / Camera_H )*0.1f;//计算每个像素代表的实际距离,单位cm;
	
	mcu_rx.rol = (s16) ((fc_att.byte_data[1]<<8) | fc_att.byte_data[0]) * 1e-2f;
	mcu_rx.pit = (s16) ((fc_att.byte_data[3]<<8) | fc_att.byte_data[2]) * 1e-2f;

		if(dot.ok)
		{ 
			dot.x = LIMIT(dot.x,-80.0f,80.0f);
			dot.y = LIMIT(dot.y,-60.0f,60.0f);
			MV.x = ( dot.x + DST*tan(mcu_rx.rol*angle_to_rad) ) *distance_l;
			MV.y = ( dot.y - DST*tan(mcu_rx.pit*angle_to_rad) ) *distance_w;
		}
		else
		{
			MV.x = 0;
			MV.y = 0;
		}	
	
//	 if(my_flight.mode == Follow_Line_Mode)
//	{
//		//对线的位置进行角度矫正,并且单位转换到mm
//		if( line.up_ok || line.down_ok )  
//		{
//				MV.x = ( line.x + DST*tan(mcu_rx.rol*angle_to_rad) ) * distance_l*10;
//		}
//		
//		if( line.left_ok || line.right_ok )
//		{
//				MV.y = ( line.y - DST*tan(mcu_rx.pit*angle_to_rad) ) * distance_w*10;
//		}
//	}
}

//视觉数据处理
void MV_Fix(float dT)
{
	//位置低通滤波
	MV.fix_x += (MV.x - MV.fix_x) * 0.2f; 
	MV.fix_y += (MV.y - MV.fix_y) * 0.2f; 
	
	//微分速度
	MV.x_s = (MV.fix_x - MV.x_o)/dT; MV.x_o = MV.fix_x;
	MV.y_s = (MV.fix_y - MV.y_o)/dT; MV.y_o = MV.fix_y;
	
	//速度低通滤波
	MV.fix_x_s += (MV.x_s - MV.fix_x_s) * 0.2f;
	MV.fix_y_s += (MV.y_s - MV.fix_y_s) * 0.2f;
	
	//速度限幅
	MV.fix_x_s = LIMIT(MV.fix_x_s,-100,100);
	MV.fix_y_s = LIMIT(MV.fix_y_s,-100,100);
}
