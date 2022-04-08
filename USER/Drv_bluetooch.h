#ifndef _DRV_BLUETOOCH_H_
#define _DRV_BLUETOOCH_H_

#include "sysconfig.h"

typedef struct 
{	
	u32  leng1;
	u32  leng;
	u32  lengnew;
	u32  lengold;
	int leng_fix;
}length;
extern length ks;

void Player_bt_Receive(u8 data);
void Bt_Duty(u8 *data_buf);
void Car_Send_Set(u8 mode);
void ks103_data(u32 data);
extern u8 one_Kland;
int kalmanfilter(int S) ;
void Send_Set_mode(u8 mode,u8 mode2);
void Car_Send(u8 mode);
#endif
