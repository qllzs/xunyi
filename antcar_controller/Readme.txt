程序编译说明：
1. 程序编译前要注意根据实际使用的电机确定宏定义的打开与关闭以及自动曝光程序的宏定义，宏定义的配置在bsp_include.h文件的开头中，OLD_WHEEL_MOTOR：为送到杭州的小车所使用的电机，MOTOR_485为使用485通信的电机，METEC_MOTOR为现在使用的最新的电机，所以北京这边的电机要使用METEC_MOTOR,  杭州小车要是使用OLD_WHEEL_MOTOR；

2. bsp_include.h 文件中BOX_CONTROL_BY_UH表示箱门电机的控制是否使用通用桥模块，杭州小车不适用，北京小车要使用；
3. 修改METEC电机的配置，右侧电机编号为1，左侧为2;如左右电机的左右反了，则需要调整CarSpeed2rpm()函数中左右轮的速度计算。
    如前后反了，则需要调整MotorSpeedSet()函数中的左右轮的转速翻转。
4.倒车的速度变为一个宏定义 BackCarSpd 在proctocal.h文件中


2018.10.12
1、增加一个debug_include.h文件，其中包含了车的一些定义，对于不用的车，需要先看该文件的配置参数，然后再进行下载程序
