#include "Drv_Beep.h"

u8 beep_flag;
void Dvr_BeepInit(void)
{
	ROM_SysCtlPeripheralEnable(BEEP_SYSCTL);
	ROM_SysCtlPeripheralEnable(LASER_SYSCTL);
	ROM_GPIOPinTypeGPIOOutput(BEEP_PORT, BEEP_PIN);
	ROM_GPIOPinTypeGPIOOutput(LASER_PORT, LASER_PIN);
  BEEP0N;
	LASER0N;
	//初始化报警系统
	Drv_BeepOnOff(0);
}

void Drv_BeepOnOff(int time0)
{
	  static int temp0,flag=0; 
	  if(time0==0)  BEEP0N;
	  else if(++temp0==time0)
            	{
								temp0=0;
							  if(flag == 0)
								{
								 BEEP0Y;
								 flag =1;
								}
								else 
								{
								 BEEP0N;
								 flag =0;
								}
              }               
}

u8 laser_flag=0;
u8 laser_TX=0;
float tracking_time=0;
u8 sum_sum;

void LASER_ctrl(float dT)
{
	if(laser_flag==1)
	{	
		  tracking_time+=dT;
			if(tracking_time>0&&tracking_time<=1)  LASER0Y;

			if(tracking_time>1&&tracking_time<=2)  LASER0N;

//			if(tracking_time>2&&tracking_time<=3)  LASER0Y;

//			if(tracking_time>3&&tracking_time<=4)  LASER0N; 
		
      if(tracking_time>2)  
			{
				sum_sum++;
				laser_flag=0;
        laser_TX = 1;				
				tracking_time=0;
			}				
	}
	
	else 
	{
		tracking_time=0;
		LASER0N;
	}
}

void beep_Ctrl(void)
{
	if(beep_flag)
	     Drv_BeepOnOff(10);
	else
		   Drv_BeepOnOff(0);
}
