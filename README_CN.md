
# Robosense-Xsens-Hikvision-Hardware-Sync
[中文README](README_CN.md)

## 1. 介绍
本项目实现了Robosense激光雷达，Xsens IMU与Hikvision工业相机之间的硬件时间同步。本项目使用的传感器为Robosense Helios-32，Xsens mti-300与Hikvision MV-CA013-21UC。

由于各个传感器使用的硬件时钟的时间不一致，且不同传感器的硬件时钟的时间流速不同，因此通常传感器驱动输出消息中的时间戳默认为电脑接收到传感器数据的系统时间。然而，传感器采样时间与电脑接收到传感器数据的系统时间之间的差值不可控，导致时间戳误与传感器采样的实际时间差较大，无法满足一些应用的需求。因此，为了统一不同传感器间的时间戳，应该将不同传感器的时间戳统一到同一时钟下。本项目中我们将激光雷达，IMU与相机的时间戳都统一到IMU时间下。

本项目基于[YangTiankai](https://github.com/YangTiankai)的项目[Hardware-Sync-of-Robosense-and-Xsense-MTI-300](https://github.com/YangTiankai/Hardware-Sync-of-Robosense-and-Xsense-MTI-300)。

## 2. 准备
### 2.1 硬件
1. 支持[StartSampling](https://base.movella.com/s/article/ClockSync-and-StartSampling)功能与[SyncOut](https://base.movella.com/s/article/Synchronization-with-the-MTi)功能的Xsens IMU，如Xsens mti-300。
2. 支持GPS+PPS时间同步与SyncOut功能的激光雷达，如Robosense Helios-32。
3. 支持外部I/O触发的相机，如Hikvision MV-CA013-21UC。
### 2.2 软件
1. ROS1 
## 3. 硬件同步原理
### 3.1 硬件连接
<div align="center">
<img src="./pics/sync.png"  width="100.0%" />
</div>

Xsens mti-300的[StartSampling](https://base.movella.com/s/article/ClockSync-and-StartSampling)功能允许IMU在接收到StartSampling信号之后再开始采样，而[SyncOut](https://base.movella.com/s/article/Synchronization-with-the-MTi)功能允许IMU输出同步脉冲。将IMU SyncOut功能的skip factor为399，使IMU每发出400个数据就发出一个同步脉冲。在stm32上电后，stm32会向IMU发出StartSampling信号。IMU接收到StartSampling信号后开始采样，并发出1Hz的PPS信号。

Robosense Helios-32支持GPS+PPS时间同步方式，接收1Hz的PPS信号与1Hz的GPRMC信号，并根据GPRMC信号中的时间设置激光雷达的硬件时间。这里我们使用stm32来伪造GPRMC信号：在stm32每次收到PPS信号时，向雷达发送一个时间增加一秒的GPRMC信号，其中stm32第一次收到PPS信号时发出的GPRMC信号中的时间为一个约定时间$T_0$ 。

Robosense Helios-32支持SyncOut输出，允许激光雷达在旋转到特定角度时输出一个脉冲。Hikvision MV-CA013-21UC允许硬件触发，在每次接收到触发信号后开始曝光。因此只要设置好激光雷达 SyncOut输出的角度，就能保证激光雷达与相机在相机的视角范围内同时采样。

### 3.2 驱动
<div align="center">
<img src="./pics/sync_time.jpeg"  width="100.0%" />
</div>

注意到stm32向激光雷达发出的GPRMC信号中的时间不是IMU的时间，而是一个从约定时间$T_0$ 开始递增的时间，因此还需要在驱动中处理时间戳。

根据[官方手册](https://base.movella.com/s/article/Synchronization-with-the-MTi)，IMU在接收到StartSampling信号的0.69ms后开始采样，并在接收到StartSampling信号的3.19ms后产生第一个加速度计/陀螺仪数据，在接收到StartSampling信号的1000.69ms后发出第一个SyncOut信号。因此，stm32接收到第一个PPS信号的时间实际上是IMU的第一个数据的1000.69ms - 3.19ms = 997.5ms后。此时stm32向激光雷达发出GPRMC信号并将激光雷达的硬件时间设置为$T_0$ 。因此在激光雷达驱动中：
$$
t_{LidarSync} = t_{Lidar} - T_0 + 997.5ms + t_{IMU0}
$$
其中$t_{LidarSync}$ 为激光雷达帧在IMU时间轴下的时间，$t_{Lidar}$ 为激光雷达帧的硬件时间，$t_{IMU0}$ 为电脑接收到第一个IMU数据的时间戳。

为了使激光雷达驱动不在相机视角范围内分帧，将激光雷达分帧的角度设置为180°。如果我们将相机的曝光时间固定为10000μS（可以根据需要自行更改），则在曝光时间内激光雷达旋转36°。为了使激光雷达与相机尽可能对齐，可以将激光雷达 Pulse Start Angle设置为342°，此时相机曝光到一半时激光雷达正好扫描正前方。由于相机在雷达旋转至18°时就结束曝光并传回图片，而激光雷达要在旋转到180°后再生成完整的激光雷达帧，因此在通讯良好的情况下，相机驱动总是先于激光雷达驱动0到100ms接收到完整的数据。因此在相机驱动中，每收到一帧图片，就等待下一个接收到的激光雷达帧，并给图片消息赋予激光雷达帧的时间戳。

## 3. 硬件
### 3.1 电路
<div align="center">
<img src="./pics/sync_board_real.png"  width="50.0%" />
</div>

本项目设计了一个pcb以方便各个传感器与stm32最小系统板之间的电气连接。各个传感器的硬件同步接口如下：

#### IMU硬件同步接口
<div align="center">
<img src="./pics/IMU_hardware1.png"  width="40.0%" />
</div>

<div align="center">
<img src="./pics/IMU_hardware.png"  width="80.0%" />
</div>

#### 激光雷达硬件同步接口
<div align="center">
<img src="./pics/Lidar_hardware1.png"  width="40.0%" />
</div>

<div align="center">
<img src="./pics/Lidar_hardware.png"  width="40.0%" />
</div>

#### 相机硬件同步接口
<div align="center">
<img src="./pics/Camera_hardware.png"  width="80.0%" />
</div>

#### 电路原理图
<div align="center">
<img src="./pics/stm32.png"  width="80.0%" />
</div>

<div align="center">
<img src="./pics/sync_board.png"  width="100.0%" />
</div>

将IMU的SyncOut连接至激光雷达的GPS_PPS。

将stm32的PA2引脚连接至IMU的SyncIn，将stm32的PA1引脚连接至IMU的SyncOut，将stm32的PA9连接至MAX232的T1IN。

将MAX232的T1OUT连接至激光雷达的GPS_GPRMC。

将激光雷达的SYNC_OUT1连接至相机的GPIO。

### 3.2 PCB
<div align="center">
<img src="./pics/pcb.png"  width="100.0%" />
</div>

| 零件  | 图片  | 数量  |
| :------------: | :------------: | :------------: |
|SH1.0-2P|<img src="./pics/sh2p.png" width=20%  />| 1 |
|SH1.0-4P|<img src="./pics/sh4p.png" width=20%  />| 1 |
|SH1.0-6P|<img src="./pics/sh6p.png" width=20%  />| 1 |
| Header20P-2.54mm |<img src="./pics/header20.jpeg" width=20%  />| 2 |
| MAX232-SOP16 |<img src="./pics/max323.jpeg" width=20%  />| 1 |
| 0.1uF-0805 |<img src="./pics/cap.jpeg" width=20%  />| 5 |

### 3.3 连接线
#### IMU数据线

<div align="center">
<img src="./pics/IMU_hardware1.png"  width="40.0%" />
</div>

| IMU航空插头（fischer 102） | USB插头/SH1.0-4P插头 |
| :------------: | :------------: |
| 引脚1 | USB GND |
| 引脚8 | USB DP(D+) |
| 引脚9 | USB DM(D-) |
| 引脚4 | USB Vcc |
| 引脚1 | SH1.0-4P Pin1 |
| 引脚5 | SH1.0-4P Pin2 |
| 引脚6 | SH1.0-4P Pin3 |
| 引脚7 | SH1.0-4P Pin4 |

#### 相机数据线

<div align="center">
<img src="./pics/Camera_hardware.png"  width="80.0%" />
</div>

| Hikvision相机触发线插头 | SH1.0-2P插头 |
| :------------: | :------------: |
| GND | Pin1 |
| GPIO | Pin2 |

#### IMU-激光雷达线

SH1.0-6P端子线10cm

### 3.4 手持设备
<div align="center">
<img src="./pics/handhold_real.jpeg"  width="50.0%" />
</div>

<div align="center">
<img src="./pics/handhold1.png"  width="80.0%" />
</div>

Lidar -> IMU Translation

$$
\left[
 \begin{matrix}
        1.0 & 0.0 & 0.0 & 0.0\\
        0.0 & 1.0 & 0.0 & -0.08\\
        0.0 & 0.0 & 1.0 & -0.0563\\
        0.0 & 0.0 & 0.0 & 1.0\\
  \end{matrix}
  \right]
$$

Lidar -> Camera Translation

$$
\left[
 \begin{matrix}
        0.0 & 0.0 & 1.0 & 0.0649\\
        -1.0 & 0.0 & 0.0 & -0.07755\\
        0.0 & -1.0 & 0.0 & -0.081\\
        0.0 & 0.0 & 0.0 & 1.0\\
  \end{matrix}
  \right]
$$

|零件|图片|数量|
| :------------: | :------------: | -------------- |
|安装板（3mm碳纤维板）|<img src="./pics/board.png" width=50%  />| 1 |
|支撑（3D打印）|<img src="./pics/support.png" width=50%  />| 1 |
|4S 2200mAh 锂电池（XT60）| <img src="./pics/battery.jpeg" width=50%  /> | 1 |
| XT60转DC 5.5 - 2.1线 | <img src="./pics/xt60-dc.jpeg" width=50%  /> | 1 |
| M3x6 D8xH8橡胶减震器 | <img src="./pics/rubber.jpeg" width=50%  /> | 4 |
| 手持三脚架 | <img src="./pics/tp97.jpeg" width=50%  /> | 1 |

## 4. 运行驱动
### 4.1 传感器设置
#### IMU设置
<div align="center">
<img src="./pics/IMU_settings1.png"  width="80.0%" />
</div>

需要在IMU驱动中打开Sample Time Fine与Sample Time Coarse。
<div align="center">
<img src="./pics/IMU_settings2.png"  width="80.0%" />
</div>

在Syncronization中添加Start Sampling功能，并设置为上升沿触发。
<div align="center">
<img src="./pics/IMU_settings3.png"  width="80.0%" />
</div>

在Syncronization中添加Sync Out功能，并设置Skip Factor为399，脉宽为100000us。

#### 激光雷达设置
<div align="center">
<img src="./pics/Lidar_settings1.png"  width="80.0%" />
</div>

在Robosense的网页驱动中打开Pulse Trigger Switch，并设置开始角度为342°，脉宽为10000000ns。
#### 相机设置

相机能够直接使用ROS驱动配置，无需提前配置。


### 4.2 运行ROS驱动

需要注意必须先连接好所有传感器，运行所有驱动，最后为激光雷达上电。

在运行驱动前，必须先设置src/mvs_ros_pkg/config/camera_trigger.yaml，src/rslidar_sdk/config/config.yaml与src/xsens_ros_mti_driver/param/xsens_mti_node.yaml中与硬件连接有关的参数，如设备SerialNumber，msop_port等

#### build
~~~shell
cd ~
git clone https://github.com/chennnununune/Robosense-Xsens-Hikvision-Hardware-Sync.git
catkin build
~~~

#### Set Env

~~~shell
cd ~/Hardware_Sync/
source devel/setup.bash
~~~

#### Launch Lidar Driver

~~~shell
roslaunch rslidar_sdk start.launch
~~~

#### Launch Camera Driver

~~~shell
roslaunch mvs_ros_pkg mvs_camera_trigger.launch
~~~

#### Launch IMU driver

~~~shell
sudo chmod 777 /dev/ttyUSB0
roslaunch xsens_mti_driver xsens_mti_node.launch
~~~
