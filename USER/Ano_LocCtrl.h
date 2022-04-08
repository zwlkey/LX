#ifndef _ANO_LOCCTRL_H
#define _ANO_LOCCTRL_H

#include "Ano_Math.h"
#include "Ano_Pid.h"
#include "Drv_mv.h"
#include "Drv_mv1.h"
#include "Drv_Yaw.h"
#include "Ano_FlightCtrl.h"

typedef struct
{
  float time;
	u8 step;
	float x;
	float y;
	float speed;

	u8 dir;
}_tracking_ctrl_st;

extern _loc_ctrl_st my_alt_fix;  //

void Loc_Ctrl(u16 dT_ms,struct _MV_ *mv);
void User_Loc_PID_Init(void);
void User_line_PID_Init(void);
void User_Len_PID_Init(void);

void Tracking_Ctrl1(float dT);//控制函数
void Tracking_Ctrl2(float dT);//控制函数
void Tracking_Ctrl3(float dT);
void Tracking_Ctrl4(float dT);
void Tracking_Ctrl5(float dT);
void Tracking_Ctrl6(float dT);


#endif
