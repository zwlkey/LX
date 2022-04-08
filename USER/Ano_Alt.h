#ifndef _ANO_ALT_H
#define _ANO_ALT_H

#include "sysconfig.h"
#include "Ano_Pid.h"
#include "Ano_FlightCtrl.h"


extern _PID_arg_st my_alt_arg_1; 
extern _PID_val_st my_alt_val_1;
extern _loc_ctrl_st my_alt_fix;

void User_Alt_PID_Init(void);

void Alt_level(float dT);

#endif

