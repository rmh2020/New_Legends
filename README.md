## 官方提供的资料位于doc文件夹内，由于部分源码已经过修改，所以以此文档为准。


C板的陀螺仪的坐标设定：

以C板平放，R标面向自己，以左下角为观测点，逆时针为正方向，对应板子的三轴




## 操作手参数：

遥控器控:

左侧按键：上 打开和关闭摩擦轮
         中 无状态
         下 从中拨到下,快速拨回中为单发
            从中拨到下,停留为连发
​		  


右侧按键：上为跟随底盘

​		  中为跟随云台

​		  下为刹车，云台静止

拨杆：

左侧拨杆控制底盘前后左右，右侧拨杆控制云台pitch和yaw，在跟随底盘模式下也控制底盘旋转。





键盘控制：将左右按键拨至中间

移动：
    W A S D 前后左右平移
​	 Q 左90度掉头 E 右 90度掉头 V 180度掉头 
    F 小陀螺 
    C 扭腰
    shift 长按 超级电容加速

射击: 
    G 单击打开摩擦轮  双击关闭摩擦轮 
    R 打开弹仓
    鼠标左键 单发 鼠标右键 连发




​	



## 硬件连接说明：

底盘电机：can2  ID 为1 2 3 4 右前，左前，左后，右后

云台电机：can1 yaw 9 pitch 10 

摩擦轮电机：can1 left 5 right 6

拨盘电机：can1 7

弹仓电机: 右侧数第三个pwm

射弹 触发条件为  BUTTEN_TRIG_PIN 为低电平 对应C板最左侧的PWM口









## 校准操作说明：

开启校准模式：    左摇杆右下，右摇杆左下  且左右按键拨至下档

陀螺仪校准           左摇杆左下，右摇杆右下  且左右按键拨至下档

云台校准               左摇杆左上，右摇杆右上  且左右按键拨至下档

底盘校准             左摇杆右上，右摇杆左上   且左右按键拨至下档