#ifndef __DEBUG_INCLUDE_H__
#define __DEBUG_INCLUDE_H__
//�������ѡ��
#define METEC_MOTOR
//485���
#define xMOTOR_485
//��챵��
#define xOLD_WHEEL_MOTOR
//ʹ��H���������ŵ��
#define BOX_CONTROL_BY_UH
//��ʹ��SD��
#define	NOSDSTATUS
//ѡ���Ƿ��������ͷ���Զ��ع⹦��
#define AUTOMATIC_EXPLORE
//���ڴ�ӡCCD����
#define Debug_CCD
#define xDebug_LineWidth
//�ڶ������̥��
#define	xVer2
//����Ѳ����ֹΪ���������Σ���⵽�����߿�Ϊһ�������ͣ��
#define HollowStopLine  
//��ͨ����λ�����ƣ��رտ��Ź�
#define	xBareDebug



/********************С���ߴ�궨��*********************/
#ifndef	Ver2
//����ֱ���궨�嵥λmm
#define D_Motor                (float)210
//С���־� mm
#define D_Car                  (float)532
#else
//��·̥
//#define D_Motor                (float)315
//����̥
#define D_Motor                (float)300
//С���־�mm
#define D_Car                  (float)533
#endif




#endif
