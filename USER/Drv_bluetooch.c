#include "Drv_bluetooch.h"
#include "Drv_Uart.h"
#include "ano_math.h"
u8 Set_Car[50];
 length ks;
 
 
extern  u8 sum_sum;
 
 
//蓝牙数据发送
void Car_Send_Set(u8 mode)     
{
	u8 _cnt=0;
	
	Set_Car[_cnt++]=0xAA;
	Set_Car[_cnt++]=0xAA;
	Set_Car[_cnt++]=0xF1; //报警帧
	Set_Car[_cnt++]=0;
	
	Set_Car[_cnt++] = mode;

	Set_Car[3] = _cnt-4;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += Set_Car[i];
	
	Set_Car[_cnt++] = sum;
	DrvUart2SendBuf(Set_Car, _cnt);
}

//蓝牙数据发送
void Car_Send(u8 mode)     
{
	u8 _cnt=0;
	
	Set_Car[_cnt++]=0xAA;
	Set_Car[_cnt++]=0xAA;
	Set_Car[_cnt++]=0xF2; //报警帧
	Set_Car[_cnt++]=0;
	
	Set_Car[_cnt++] = mode;

	Set_Car[3] = _cnt-4;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += Set_Car[i];
	
	Set_Car[_cnt++] = sum;
	DrvUart2SendBuf(Set_Car, _cnt);
}



u8 Set_Mode[50];


void Send_Set_mode(u8 mode,u8 mode2)     
{
	u8 _cnt=0;
	
	Set_Mode[_cnt++]=0xAA;
	Set_Mode[_cnt++]=0xAA;
	Set_Mode[_cnt++]=0xF3; //报警帧
	Set_Mode[_cnt++]=0;
	
	Set_Mode[_cnt++] = mode>>8;
	Set_Mode[_cnt++] = mode;
	Set_Mode[_cnt++] = mode2>>8;
	Set_Mode[_cnt++] = mode2;
	
	Set_Mode[3] = _cnt-4;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += Set_Car[i];
	
	Set_Mode[_cnt++] = sum;
	DrvUart2SendBuf(Set_Mode, _cnt);
}


//c8t6数据接收
void Player_bt_Receive(u8 data) //串口二
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
			Bt_Duty(RxBuffer);
		break;	

		default:	state = 0;	break;
	}
}
u8 one_Kland=0,i=0;
u32 lengold=250;
u32 sum;


void ks103_data(u32 data)
{     
	  i++;
	  ks.leng_fix = ks.lengnew - lengold;
		     lengold = ks.lengnew;
		     if(ABS(ks.leng_fix)<=10)
			    ks.leng1=ks.lengnew;
			 sum+=ks.leng1;
			 if(i==15)
			 {
				 ks.leng=1.0*sum/15*1;
				 sum=0;
				 i=0;
			 }	
	 ks.leng = kalmanfilter(ks.leng);		 
}

int kalmanfilter(int S)  
{
  static  float x_mid,x_last=0,p_mid,p_last=10,p_now;
	static  float Q=0.225,R=0.780; 
	static  float Kg;  
	int Output;
	
	x_mid = x_last;
	p_mid = p_last + Q;
	Kg=p_mid/(p_mid+R);
	Output=(long)x_mid + Kg*(S-x_mid);	
	p_now = (1-Kg)*p_mid;
	p_last = p_now;
	x_last = Output;
	return Output;
}


u8 Pro_Ctr_step;

//接收数据解析

void Bt_Duty(u8 *data_buf)
{
	switch( *(data_buf+2) )    
	{
		
		case 0xF0:             //超声波数据接收
	         ks.lengnew=1.0f*((u16)(*(data_buf+4)<<8)|*(data_buf+5));
		break;

		case 0xF2:
			    
		       one_Kland = (u16) (*(data_buf+4));
			
		break;
		
		case 0xF3:
		      Pro_Ctr_step = (u16)(*(data_buf+4)<<8)|*(data_buf+5);
		break;
				
	}
	
}

