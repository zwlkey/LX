#ifndef _ANO_FLIGHTCTRL_H
#define _ANO_FLIGHTCTRL_H

#include "sysconfig.h"

void ALL_PID_Init(void);

void Flight_Mode_Set(u32 dT_us);
	

enum
{
	X = 0,
	Y = 1,
	Z = 2,
	VEC_XYZ,
};

typedef struct
{
	float exp[VEC_XYZ];
	float fb[VEC_XYZ];

	
	float out[VEC_XYZ];
}_loc_ctrl_st;

struct MYMODE
{
    u8 mode;
	u8 mode_old;
	u8 color;
	u8 color_old;
	u8 mode1;
	u8 mode_old1;
	u8 color1;
	u8 color_old1;
};

struct _rol_pit_
{
	s16 rol;
	s16 pit;
	s16 yaw;
};

enum _my_mode
{
	Follow_Null = 0,
	Follow_Dot_Mode,
	Follow_Line_Mode,
	Follow_RGDot_Mode
};


#endif
