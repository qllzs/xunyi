#ifndef __DEBUG_INCLUDE_H__
#define __DEBUG_INCLUDE_H__
//电机类型选择
#define METEC_MOTOR
//485电机
#define xMOTOR_485
//轮毂电机
#define xOLD_WHEEL_MOTOR
//使用H桥驱动箱门电机
#define BOX_CONTROL_BY_UH
//不使用SD卡
#define	NOSDSTATUS
//选择是否进行摄像头的自动曝光功能
#define AUTOMATIC_EXPLORE
//串口打印CCD数据
#define Debug_CCD
#define xDebug_LineWidth
//第二版大轮胎车
#define	xVer2
//定义巡线中止为空心三角形，检测到两条线宽为一定距离后停车
#define HollowStopLine  
//不通过上位机控制，关闭看门狗
#define	xBareDebug



/********************小车尺寸宏定义*********************/
#ifndef	Ver2
//轮子直径宏定义单位mm
#define D_Motor                (float)210
//小车轮距 mm
#define D_Car                  (float)532
#else
//公路胎
//#define D_Motor                (float)315
//场地胎
#define D_Motor                (float)300
//小车轮距mm
#define D_Car                  (float)533
#endif




#endif
