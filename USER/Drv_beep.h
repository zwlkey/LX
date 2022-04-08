#ifndef _DRV_BEEP_H_
#define _DRV_BEEP_H_

#include "sysconfig.h"

#define LASER0Y ROM_GPIOPinWrite(LASER_PORT, LASER_PIN, LASER_PIN)
#define LASER0N ROM_GPIOPinWrite(LASER_PORT, LASER_PIN, 0)

#define BEEP0Y ROM_GPIOPinWrite(BEEP_PORT, BEEP_PIN, BEEP_PIN)
#define BEEP0N ROM_GPIOPinWrite(BEEP_PORT, BEEP_PIN, 0)

void Dvr_BeepInit(void);
void Drv_BeepOnOff(int time0);
void beep_Ctrl(void);
void LASER_ctrl(float dT);
#endif
