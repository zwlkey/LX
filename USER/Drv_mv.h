#ifndef _DRV_MV_H_
#define _DRV_MV_H_

#include "sysconfig.h"

#define CHECK_NULL 0	//������
#define CHECK_DOT  1	//����
#define CHECK_LINE 2  //�߼��
#define CHECK_LINE_DOT 3 //�����
#define CHECK_RGDOT 4 //����

//ʵ�ʳߴ�  //1m
#define Camera_H 1000.0f
#define Pictur_L 880.0f   //��
#define Pictur_W 660.0f   //��

//ʵ������
#define DST 180
#define HIG 120
#define WID 160

#define angle_to_rad 0.0174f  //jiaodyu

#define NUL  0  //
#define RED  1
#define BLK  2
#define GRN  3
#define YAL  4

//�������ݽṹ��
struct _dot_check_
{
	float x;
	float y;
	u8 flag;
	u8 ok;
};


struct _MV_
{
	float x;
	float y;
	float x_s;
	float y_s;
	float x_o;
	float y_o;
	float fix_x;
	float fix_y;
	float fix_x_s;
	float fix_y_s;
	
	u8 x_ok,y_ok;
	u8 line_mode,dot_mode,qr_mode,bar_mode,tag_mode;
	u8 mode,set_mode,sys_mode,user_mode;
	
	float test_0,test_1;
};

//�߼�����ݽṹ��
struct _line_check_
{
	float x;
	float y;
	float angle_x;
	float angle_y;
	float angle;
	float tag_x;
	float cross_x;
	float cross_y;
	
	u8 angle_ok;
	u8 leng_ok;
	u8 tag_ok;
	u8 cross_ok;
	u8 flag;
};
void Player_Duty(u8 *data_buf);
void Player_Mv_Receive(u8 data);
void Player_Send_Set(u8 mode,u8 color);
void MV_Flag_Check(u8 flag0,u8 flag1);
void MV_Fix(float dT);
void MV_Decoupling(u8 dT_ms);//��ת����
#endif
