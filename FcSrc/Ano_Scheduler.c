/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * ����    �������ƴ�
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ����    ���������
**********************************************************************************/
#include "Ano_Scheduler.h"
#include "User_Task.h"
//////////////////////////////////////////////////////////////////////
//�û����������
//////////////////////////////////////////////////////////////////////

#include "Drv_mv1.h"
#include "Drv_mv.h"
#include "Drv_Yaw.h"
#include "Ano_LocCtrl.h"
#include "Ano_Flightctrl.h"
#include "Drv_Beep.h"
#include "Ano_Alt.h"

extern struct _line_check_ line;
extern struct _dot_check_ dot_mv1;
extern struct _line_check_ line_mv1;
extern struct _dot_check_ dot;

static void Loop_1000Hz(void) //1msִ��һ��
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_500Hz(void) //2msִ��һ��
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_200Hz(void) //5msִ��һ��
{
	//////////////////////////////////////////////////////////////////////
    User_my_yaw_2level(5);   //Ѳ�߽Ƕ�����
	//////////////////////////////////////////////////////////////////////
}

static void Loop_100Hz(void) //10msִ��һ��
{
	//////////////////////////////////////////////////////////////////////
   Flight_Mode_Set(1*1e-2f);    //ģʽ�л�
	//////////////////////////////////////////////////////////////////////
}

extern struct _MV_ MV;

static void Loop_50Hz(void) //20msִ��һ��
{
	//////////////////////////////////////////////////////////////////////
	Loc_Ctrl(20,&MV); 
	MV_Decoupling(20);
	beep_Ctrl();
//	Alt_level(20);
	Tracking_Ctrl1(0.02f);
	Tracking_Ctrl2(0.02f);
	Tracking_Ctrl3(0.02f);
	Tracking_Ctrl4(0.02f);
	Tracking_Ctrl5(0.02f);
	Tracking_Ctrl6(0.02f);	
	
	//////////////////////////////////////////////////////////////////////
}

static void Loop_20Hz(void) //50msִ��һ��
{
	//////////////////////////////////////////////////////////////////////
 LASER_ctrl(0.05);
	//////////////////////////////////////////////////////////////////////
}

static void Loop_2Hz(void) //500msִ��һ��
{
	
}
//////////////////////////////////////////////////////////////////////
//��������ʼ��
//////////////////////////////////////////////////////////////////////
//ϵͳ�������ã�������ִͬ��Ƶ�ʵġ��̡߳�
static sched_task_t sched_tasks[] =
	{
		{Loop_1000Hz, 1000, 0, 0},
		{Loop_500Hz, 500, 0, 0},
		{Loop_200Hz, 200, 0, 0},
		{Loop_100Hz, 100, 0, 0},
		{Loop_50Hz, 50, 0, 0},
		{Loop_20Hz, 20, 0, 0},
		{Loop_2Hz, 2, 0, 0},
};
//�������鳤�ȣ��ж��߳�����
#define TASK_NUM (sizeof(sched_tasks) / sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//��ʼ�������
	for (index = 0; index < TASK_NUM; index++)
	{
		//����ÿ���������ʱ������
		sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;
		//�������Ϊ1��Ҳ����1ms
		if (sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//��������ŵ�main������while(1)�У���ͣ�ж��Ƿ����߳�Ӧ��ִ��
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//ѭ���ж������̣߳��Ƿ�Ӧ��ִ��

	for (index = 0; index < TASK_NUM; index++)
	{
		//��ȡϵͳ��ǰʱ�䣬��λMS
		uint32_t tnow = GetSysRunTimeMs();
		//�����жϣ������ǰʱ���ȥ��һ��ִ�е�ʱ�䣬���ڵ��ڸ��̵߳�ִ�����ڣ���ִ���߳�
		if (tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{

			//�����̵߳�ִ��ʱ�䣬������һ���ж�
			sched_tasks[index].last_run = tnow;
			//ִ���̺߳�����ʹ�õ��Ǻ���ָ��
			sched_tasks[index].task_func();
		}
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
