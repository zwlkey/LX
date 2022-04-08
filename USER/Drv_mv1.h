#ifndef _DRV_MV1_H_
#define _DRV_MV1_H_

#include "sysconfig.h"

extern struct _line_check_ line_mv1;
extern struct _dot_check_ dot_mv1;
extern struct _MV_ 		MV_mv1;

void Player1_Duty(u8 *data_buf);
void Player_Mv1_Receive(u8 data);
void MV1_Flag_Check(u8 flag0,u8 flag1);
void Player_Sendmv1_Set(u8 mode,u8 color);
void MV1_Send(u8 mode);
#endif
