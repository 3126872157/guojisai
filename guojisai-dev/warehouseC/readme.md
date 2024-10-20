I2C3：磁力计

SPI1：IMU

TIM4：蜂鸣器，APB1：84Hz

TIM10：IMU温控电阻，APB1

TIM1：pwm舵机，APB2：168Hz

ch1：拨杆，pulse：250

ch2：凸轮，pulse：250

ch3：滑道，pulse：800

UART1（外壳丝印UART2）：4.8M波特率，PA9，PB7

PE14（TIM1CH4）：IO串口Rx，给IC卡用

PI6，PI7（PWM最后两路）灰度1，2

USB：OTG_FS开device only，USB_DEVICE：VCP

在usbd_cdc_if.c里的CDC_Receive_FS函数处理视觉数据

IC卡读取：PE14为外部中断下降沿触发同时可以读电平，TIM2波特率频率，TIM3产生us延时

通过IO_Serial.c逐字节读取IC卡的数据