#include "Drv_mv1.h"
#include "Drv_mv.h"
#include "Ano_Math.h"
#include "math.h"
#include "Drv_Uart.h"
#include "Ano_FlightCtrl.h"

extern struct _line_check_ line_mv1;
extern struct _dot_check_ dot_mv1;
extern struct MYMODE my_flight_mv1;
u8 Set_Mv1_Mode[50];
u8 Set_PTOTOmv1_Mode[50];

////拍照命令
//void MV1_Send(u8 mode)
//{
//	u8 _cnt=0;
//	
//	Set_PTOTOmv1_Mode[_cnt++]=0xAA;
//	Set_PTOTOmv1_Mode[_cnt++]=0xAF;
//	Set_PTOTOmv1_Mode[_cnt++]=0xF0;
//	Set_PTOTOmv1_Mode[_cnt++]=0;
//	
//	Set_PTOTOmv1_Mode[_cnt++] = mode;
//	Set_PTOTOmv1_Mode[3] = _cnt-4;
//	
//	u8 sum = 0;
//	for(u8 i=0;i<_cnt;i++)
//		sum += Set_PTOTOmv1_Mode[i];
//	
//	Set_PTOTOmv1_Mode[_cnt++] = sum;
//	DrvUart1SendBuf(Set_PTOTOmv1_Mode, _cnt);
//}

//视觉定位设置
void Player_Sendmv1_Set(u8 mode,u8 color)
{
	u8 _cnt=0;
	
	Set_Mv1_Mode[_cnt++]=0xAA;
	Set_Mv1_Mode[_cnt++]=0xAF;
	Set_Mv1_Mode[_cnt++]=0xF1;
	Set_Mv1_Mode[_cnt++]=0;
	
	Set_Mv1_Mode[_cnt++] = mode;
  Set_Mv1_Mode[_cnt++] = color;
	Set_Mv1_Mode[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += Set_Mv1_Mode[i];
	
	Set_Mv1_Mode[_cnt++] = sum;
	DrvUart1SendBuf(Set_Mv1_Mode, _cnt);
}

//Openmv数据接收
void Player_Mv1_Receive(u8 data)
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
			Player1_Duty(RxBuffer);
		break;	

		default:	state = 0;	break;
	}
}

//视觉标志位检测
void MV1_Flag_Check(u8 flag0,u8 flag1)
{
	static u8 cnt0,cnt4;
	static u8 cnt0_,cnt4_;

	//线检测标志位
	if( flag0 ==1 ){cnt0++;cnt0_=0; if(cnt0>200)cnt0=200;} else {cnt0=0;cnt0_++;if(cnt0_>200)cnt0_=200;}
	if(cnt0>3) line_mv1.angle_ok    = 1; else if(cnt0_>10) line_mv1.angle_ok    = 0;//判断线检测

	//点检测标志位
	 if( flag1 ){cnt4++;cnt4_=0; if(cnt4>200)cnt4=200;} else {cnt4=0;cnt4_++;if(cnt4_>200)cnt4_=200;}
	 if(cnt4>30) dot_mv1.ok = 1; else if(cnt4_>30) dot_mv1.ok = 0;//判断是否检测到点
}

//视觉定位数据解析
void Player1_Duty(u8 *data_buf)
{
	switch( *(data_buf+2) )
	{	
		case 0xF2:
			
		   dot_mv1.flag =  *(data_buf+4);
		
		case 0xF3://线检测数据	
				line_mv1.flag = *(data_buf+8);
		    line_mv1.angle_x = ((s16)(*(data_buf+4)<<8)|*(data_buf+5));   line_mv1.angle_x *= 0.01f;
		    line_mv1.angle_y = ((s16)(*(data_buf+6)<<8)|*(data_buf+7));		line_mv1.angle_y *= 0.01f;
		  break;
		
		case 0xF4://线加点检测数据
				line_mv1.flag = *(data_buf+4);
				line_mv1.tag_x =   ((s16)(*(data_buf+5)<<8)|*(data_buf+6));
				line_mv1.cross_x = ((s16)(*(data_buf+7)<<8)|*(data_buf+8));
		  break;
	}	
}
