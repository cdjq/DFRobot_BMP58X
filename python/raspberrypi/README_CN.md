# DFRobot_BMP58X

* [English Version](./README.md)

这是一个BMP58X的库，功能是读取温度和压力。BMP(585/581)是一款基于可靠传感原理的压力和温度测量数字传感器。

![正反面svg效果图]()


## 产品链接（链接到英文商城）
   
## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)

## 概述

* 该库支持BMP585/581传感器。
* 该库支持读取温度和压力。
* 该库支持设置传感器的工作模式。
* 该库支持设置传感器的输出数据率。
* 该库支持设置传感器的过采样率。
* 该库支持设置传感器的IIR滤波器系数。
* 该库支持设置传感器的FIFO操作。
* 该库支持设置传感器的中断行为。
* 该库支持设置传感器的压力超限检测。
* 该库支持读取传感器的中断状态。

## 库安装

使用此库前，请首先下载库文件，将其粘贴到\Arduino\libraries目录中，然后打开examples文件夹并在该文件夹中运行演示。

## 方法

```python
def begin(self):
'''!
  @fn begin
  @brief 初始化传感器硬件接口
  @return 初始化成功返回true，失败返回false
'''

def set_odr(self, odr):
'''!
  @fn set_odr
  @brief 配置传感器输出数据速率(ODR)
  @param odr 输出数据速率选择
  @n 可用速率：
  @n - eODR_240_HZ:    240 Hz
  @n - eODR_218_5_HZ:  218.5 Hz
  @n - eODR_199_1_HZ:  199.1 Hz
  @n - eODR_179_2_HZ:  179.2 Hz
  @n - eODR_160_HZ:    160 Hz
  @n - eODR_149_3_HZ:  149.3 Hz
  @n - eODR_140_HZ:    140 Hz
  @n - eODR_129_8_HZ:  129.8 Hz
  @n - eODR_120_HZ:    120 Hz
  @n - eODR_110_1_HZ:  110.1 Hz
  @n - eODR_100_2_HZ:  100.2 Hz
  @n - eODR_89_6_HZ:   89.6 Hz
  @n - eODR_80_HZ:     80 Hz
  @n - eODR_70_HZ:     70 Hz
  @n - eODR_60_HZ:     60 Hz
  @n - eODR_50_HZ:     50 Hz
  @n - eODR_45_HZ:     45 Hz
  @n - eODR_40_HZ:     40 Hz
  @n - eODR_35_HZ:     35 Hz
  @n - eODR_30_HZ:     30 Hz
  @n - eODR_25_HZ:     25 Hz
  @n - eODR_20_HZ:     20 Hz
  @n - eODR_15_HZ:     15 Hz
  @n - eODR_10_HZ:     10 Hz
  @n - eODR_05_HZ:     5 Hz
  @n - eODR_04_HZ:     4 Hz
  @n - eODR_03_HZ:     3 Hz
  @n - eODR_02_HZ:     2 Hz
  @n - eODR_01_HZ:     1 Hz
  @n - eODR_0_5_HZ:    0.5 Hz
  @n - eODR_0_250_HZ:  0.250 Hz
  @n - eODR_0_125_HZ:  0.125 Hz
  @return 配置成功返回true，失败返回false
'''

def set_osr(self, osr_t, osr_p):
'''!
  @fn set_osr
  @brief 设置温度和压力的过采样率
  @param osr_temp 温度过采样
  @param osr_press 压力过采样
  @n 支持的值：
  @n - OVERSAMPLING_1X:   1倍过采样
  @n - OVERSAMPLING_2X:   2倍过采样
  @n - OVERSAMPLING_4X:   4倍过采样
  @n - OVERSAMPLING_8X:   8倍过采样
  @n - OVERSAMPLING_16X:  16倍过采样
  @n - OVERSAMPLING_32X:  32倍过采样
  @n - OVERSAMPLING_64X:  64倍过采样
  @n - OVERSAMPLING_128X: 128倍过采样
  @return 配置成功返回true，失败返回false
'''

def set_measure_mode(self, mode):
'''!
  @fn setMeasureMode
  @brief 设置传感器的测量模式
  @param mode 测量模式
  @n      SLEEP_MODE = 0x00.        #// 待机模式
  @n      NORMAL_MODE = 0x01.        #// 正常模式
  @n      SINGLE_SHOT_MODE = 0x02.         #// 单次模式 > 仅执行一次
  @n      CONTINOUS_MODE = 0x03.      #// 连续模式
  @n      DEEP_SLEEP_MODE = 0x04.   #// 深度待机模式
  @return 设置成功返回True，否则返回False
'''

def reset(self):
'''!
  @fn reset
  @brief 复位传感器
  @return 复位成功返回True，否则返回False
'''

def get_temperature(self):
'''!
  @fn get_temperature
  @brief 获取传感器温度
  @return 温度值（摄氏度）
'''

def get_pressure(self):
'''!
  @fn get_pressure
  @brief 获取传感器压力
  @return 压力值（帕斯卡）
'''

def get_altitude(self):
'''!
  @fn get_altitude
  @brief 获取传感器海拔高度
  @return 海拔高度（米）
'''

def config_iir(self, iir_t, iir_p):
'''!
  @fn config_iir
  @brief 配置IIR滤波器系数
  @param iir_t 温度IIR滤波器
  @param iir_p 压力IIR滤波器
  @n 可用系数：
  @n - IIR_FILTER_BYPASS:   旁路滤波器
  @n - IIR_FILTER_COEFF_1:  1阶滤波器
  @n - IIR_FILTER_COEFF_3:  3阶滤波器
  @n - IIR_FILTER_COEFF_7:  7阶滤波器
  @n - IIR_FILTER_COEFF_15: 15阶滤波器
  @n - IIR_FILTER_COEFF_31: 31阶滤波器
  @n - IIR_FILTER_COEFF_63: 63阶滤波器
  @n - IIR_FILTER_COEFF_127:127阶滤波器
  @return 配置成功返回True，否则返回False
'''

def config_fifo(self, frame_sel=eFIFO_PRESS_TEMP_DATA, dec_sel=eFIFO_NO_DOWNSAMPLING, mode=eFIFO_STREAM_TO_FIFO_MODE, threshold=0x00):
'''!
  @fn config_fifo
  @brief 配置FIFO操作参数
  @param frame_sel 数据帧类型
  @n 可用类型：
  @n - FIFO_DISABLED:    FIFO禁用
  @n - FIFO_TEMPERATURE_DATA: 仅温度数据
  @n - FIFO_PRESSURE_DATA:    仅压力数据
  @n - FIFO_PRESS_TEMP_DATA:  压力和温度数据

  @param dec_sel 降采样比率
  @n 可用比率：
  @n - FIFO_NO_DOWNSAMPLING: 无降采样
  @n - FIFO_DOWNSAMPLING_2X:  2倍降采样
  @n - FIFO_DOWNSAMPLING_4X:  4倍降采样
  @n - FIFO_DOWNSAMPLING_8X:  8倍降采样
  @n - FIFO_DOWNSAMPLING_16X: 16倍降采样
  @n - FIFO_DOWNSAMPLING_32X: 32倍降采样
  @n - FIFO_DOWNSAMPLING_64X: 64倍降采样
  @n - FIFO_DOWNSAMPLING_128X:128倍降采样

  @param mode FIFO操作模式
  @n 可用模式：
  @n - FIFO_STREAM_TO_FIFO_MODE: 连续流数据
  @n - FIFO_STOP_ON_FULL_MODE:   FIFO满时停止

  @param threshold FIFO触发阈值(0=禁用, 1-31=帧数)]
  @n - 0x0F: 15帧。这是PT模式下的最大设置。最高位被忽略。
  @n - 0x1F: 31帧。这是P或T模式下的最大设置。
  @return 配置成功返回True，否则返回False
'''

def get_fifo_count(self):
'''!
  @fn get_fifo_count
  @brief 获取FIFO中的帧数
  @return FIFO中的帧数
'''

def get_fifo_data(self):
'''!
  @fn getFIFOData
  @brief 从FIFO读取所有数据
  @return sFIFOData_t 包含压力和温度数据的结构体
  @n - len: 存储的数据帧数(0-31)
  @n - pressure: 压力值数组
  @n - temperature: 温度值数组
'''

def config_interrupt(self, int_mode, int_pol, int_od):
'''!
  @fn config_interrupt
  @brief 配置中断行为
  @param int_mode 触发模式
  @n 可用模式：
  @n - INT_MODE_PULSED: 脉冲模式
  @n - INT_MODE_LATCHED: 锁存模式

  @param int_pol 信号极性
  @n 可用极性：
  @n - INT_POL_ACTIVE_LOW: 低电平有效
  @n - INT_POL_ACTIVE_HIGH: 高电平有效

  @param int_od 输出驱动类型
  @n 可用类型：
  @n - INT_OD_PUSH_PULL: 推挽输出
  @n - INT_OD_OPEN_DRAIN: 开漏输出
  @return 配置成功返回True，否则返回False
'''

def set_int_source(self, source):
'''!
  @fn set_int_source
  @brief 使能特定中断源
  @param source 触发器的位掩码
  @n 可用源：
  @n - eINT_DATA_DRDY:    数据就绪中断
  @n - eINT_FIFO_FULL:    FIFO满中断
  @n - eINT_FIFO_THRES:   FIFO阈值中断
  @n - eINT_PRESSURE_OOR: 压力超范围中断
  @return 配置成功返回True，否则返回False
'''

def get_int_status(self):
'''!
  @brief 获取传感器的中断状态
  @return 中断状态
  @n      eINT_STATUS_DRDY = 0x01,                   # // 数据就绪
  @n      eINT_STATUS_FIFO_FULL = 0x02,              # // FIFO满
  @n      eINT_STATUS_FIFO_THRES = 0x04,             # // FIFO阈值
  @n      eINT_STATUS_PRESSURE_OOR = 0x08,           # // 压力超范围
  # // 上电复位/软复位完成
  @n      eINT_STATUS_POR_SOFTRESET_COMPLETE = 0x10,
'''

def set_oor_press(self, oor, range_val, cnt_lim):
'''!
  @brief 设置传感器的超范围压力
  @param oor 超范围压力值
  @n      0x00000 - 0x1FFFF: 超范围压力值
  @param range_val 超范围压力范围
  @n      0x00 - 0xFF: 超范围压力范围 (oor - range_val, oor + range_val)
  @param cnt_lim 超范围压力计数限制
  @n      eOOR_COUNT_LIMIT_1  = 0x00
  @n      eOOR_COUNT_LIMIT_3  = 0x01
  @n      eOOR_COUNT_LIMIT_7  = 0x02
  @n      eOOR_COUNT_LIMIT_15 = 0x03
  @return 配置成功返回True，否则返回False
'''

def calibrated_absolute_difference(self, altitude):
'''!
  @fn calibratedAbsoluteDifference
  @brief 使用给定的当前海拔高度作为参考值，消除后续压力和海拔数据的绝对差异
  @param altitude 当前海拔高度
  @return 布尔值，指示参考值是否设置成功
  @retval True 表示参考值设置成功
  @retval False 表示设置参考值失败
'''

def set_baud(self, baud):
'''!
  @fn setBaud
  @brief 设置UART通信波特率
  @details 使用指定的波特率枚举值配置串行通信速度。
          该函数初始化必要的硬件寄存器以实现所需的数据传输速率。
  @param baud eBaud枚举值，指定所需的波特率。
              如果未显式设置，默认为e9600。
  @note 实际硬件配置可能因微控制器型号而异。
        该函数假设使用标准时钟频率；如果使用非默认系统时钟配置，请调整时钟设置。
  @warning 在通信过程中更改波特率可能导致数据丢失或通信错误（如果两个设备未同步）。
  @see eBaud可用的波特率选项：
      - BAUD_2400: 2400比特/秒
      - BAUD_4800: 4800比特/秒
      - BAUD_9600: 9600比特/秒（默认值）
      - BAUD_14400: 14400比特/秒
      - BAUD_19200: 19200比特/秒
      - BAUD_38400: 38400比特/秒
      - BAUD_57600: 57600比特/秒
      - BAUD_115200: 115200比特/秒
'''
```

## 兼容性

* RaspberryPi Version

				
| 板子型号      |  运行良好  | 运行异常  | 未测试  | 备注 |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

* Python Version

| Python  |  运行良好  | 运行异常  | 未测试  | 备注 |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |           |            |     √    |         |
| Python3 |     √     |            |          |         |

## 历史

- Data 2025-09-23
- Version V1.0.0

## 创作者

Written by(yuanlong.yu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
