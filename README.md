
# Robosense-Xsens-Hikvision-Hardware-Sync
[中文README](README_CN.md)

## 1. Description
This project implements hardware time synchronization between the Robosense LiDAR, Xsens IMU, and Hikvision industrial camera. The sensors used in this project are the Robosense Helios-32, Xsens MTi-300, and Hikvision MV-CA013-21UC.

Due to the inconsistency of hardware clock times used by various sensors, and the different rates at which these clocks progress, sensor driver output messages typically use the system time when the computer receives the sensor data as their timestamps. However, the difference between the sensor sampling time and the system time when the data is received is unpredictable, leading to significant discrepancies between the timestamps and the actual sampling times. This can be problematic for certain applications. Therefore, to synchronize timestamps across different sensors, they should be unified under the same clock. In this project, we synchronize the timestamps of the LiDAR, IMU, and camera to the IMU clock.

This project is based on [YangTiankai's project](https://github.com/YangTiankai) titled [Hardware-Sync-of-Robosense-and-Xsense-MTI-300](https://github.com/YangTiankai/Hardware-Sync-of-Robosense-and-Xsense-MTI-300)。

## 2. Requirements
### 2.1 Hardware
1. Xsens IMUs with [StartSampling](https://base.movella.com/s/article/ClockSync-and-StartSampling) and [SyncOut](https://base.movella.com/s/article/Synchronization-with-the-MTi)features.
2. Lidar with GPS+PPS time-sync and SyncOut features.
3. Camera with external trigger feature.
### 2.2 Software
1. ROS1 
## 3. Time-Sync Explanation
### 3.1 Connection
<div align="center">
<img src="./pics/sync.png"  width="100.0%" />
</div>
The [StartSampling](https://base.movella.com/s/article/ClockSync-and-StartSampling) function of the Xsens MTi-300 allows the IMU to start sampling only after receiving the StartSampling signal. The [SyncOut](https://base.movella.com/s/article/Synchronization-with-the-MTi) function enables the IMU to output synchronization pulses. By setting the IMU SyncOut function's skip factor to 399, the IMU will output one synchronization pulse for every 400 data samples. Once the STM32 is powered on, it sends a StartSampling signal to the IMU. After receiving the StartSampling signal, the IMU begins sampling and outputs a 1Hz PPS (Pulse Per Second) signal.

The Robosense Helios-32 supports GPS+PPS time synchronization by receiving a 1Hz PPS signal and a 1Hz GPRMC signal. It sets the LiDAR hardware time based on the time information from the GPRMC signal. In this setup, we use an STM32 to simulate the GPRMC signal: each time the STM32 receives a PPS signal, it sends a GPRMC signal to the LiDAR with the time incremented by one second. The first GPRMC signal sent by the STM32 when it receives the first PPS signal will contain a pre-defined time $T_0$.

The Robosense Helios-32 supports SyncOut output, allowing the LiDAR to send a pulse when it rotates to a specific angle. The Hikvision MV-CA013-21UC supports hardware trigger, starting exposure each time a trigger signal is received. Hence, by configuring the SyncOut output angle of the LiDAR, the LiDAR and camera will sample simultaneously within the camera's field of view.

### 3.2 Driver
<div align="center">
<img src="./pics/sync_time.jpeg"  width="100.0%" />
</div>
Note that the time in the GPRMC signal sent from the STM32 to the LiDAR is not the IMU time, but rather a time that starts increasing from a pre-defined time $T_0$. Therefore, timestamp handling is also required in the driver.

According to the [official manual](https://base.movella.com/s/article/Synchronization-with-the-MTi), the IMU starts sampling 0.69 ms after receiving the StartSampling signal and generates the first accelerometer/gyroscope data 3.19 ms after receiving the StartSampling signal. The first SyncOut signal is send 1,000.69 ms after receiving the StartSampling signal. Therefore, the stm32 receives the first PPS signal 1,000.69 ms - 3.19 ms = 997.5 ms after the IMU's first data. At this point, the stm32 sends the GPRMC signal to the Lidar and sets the Lidar's hardware time to $T_0$. Therefore, in the Lidar driver:
$$
t_{LidarSync} = t_{Lidar} - T_0 + 997.5ms + t_{IMU0}
$$
Where $t_{LidarSync}$ is the time of the LiDAR frame in the IMU timeline, $t_{Lidar}$ is the hardware time of the LiDAR frame, $t_{IMU0}$ is the timestamp of the first IMU data received by the computer.

To prevent LiDAR frame splitting within the camera's field of view, set the LiDAR's frame-splitting angle to 180°. If we fix the camera's exposure time to 10,000 μs (which can be adjusted as needed), the LiDAR will rotate 36° during this exposure period. To align the LiDAR with the camera as closely as possible, you can set the LiDAR Pulse Start Angle to 342°, so that the LiDAR is positioned at 0° when the camera exposure is halfway through. Since the camera finishes exposure and returns the image when the LiDAR reaches 18°, but the LiDAR frame is only completed after rotating to 180°, the camera typically receives the complete data 0 to 100 ms earlier than the LiDAR. Therefore, in the camera driver, after receiving an image frame, the system waits for the next LiDAR frame and assigns the LiDAR frame's timestamp to the image message.

## 3. Hardware
### 3.1 Circuit
<div align="center">
<img src="./pics/sync_board_real.png"  width="50.0%" />
</div>
This project designed a PCB to facilitate the electrical connection between the various sensors and the STM32 minimum system board. The hardware sync interfaces for each sensor are as follows:

#### IMU Interface
<div align="center">
<img src="./pics/IMU_hardware1.png"  width="40.0%" />
</div>
<div align="center">
<img src="./pics/IMU_hardware.png"  width="80.0%" />
</div>

#### Lidar Interface
<div align="center">
<img src="./pics/Lidar_hardware1.png"  width="40.0%" />
</div>
<div align="center">
<img src="./pics/Lidar_hardware.png"  width="40.0%" />
</div>

#### Camera Interface
<div align="center">
<img src="./pics/Camera_hardware.png"  width="80.0%" />
</div>

#### Circuit Schematic
<div align="center">
<img src="./pics/stm32.png"  width="80.0%" />
</div>
<div align="center">
<img src="./pics/sync_board.png"  width="100.0%" />
</div>

Connect the IMU's SyncOut to the LiDAR's GPS_PPS.

Connect the STM32's PA2 pin to the IMU's SyncIn, the STM32's PA1 pin to the IMU's SyncOut, and the STM32's PA9 pin to the MAX232's T1IN.

Connect the MAX232's T1OUT to the LiDAR's GPS_GPRMC.

Connect the LiDAR's SYNC_OUT1 to the Camera's GPIO.

### 3.2 PCB
<div align="center">
<img src="./pics/pcb.png"  width="100.0%" />
</div>
| Component | Pics | Num |
| :------------: | :------------: | :------------: |
|SH1.0-2P|<img src="./pics/sh2p.png" width=20%  />| 1 |
|SH1.0-4P|<img src="./pics/sh4p.png" width=20%  />| 1 |
|SH1.0-6P|<img src="./pics/sh6p.png" width=20%  />| 1 |
| Header20P-2.54mm |<img src="./pics/header20.jpeg" width=20%  />| 2 |
| MAX232-SOP16 |<img src="./pics/max323.jpeg" width=20%  />| 1 |
| 0.1uF-0805 |<img src="./pics/cap.jpeg" width=20%  />| 5 |

### 3.3 Cable
#### IMU Cable

<div align="center">
<img src="./pics/IMU_hardware1.png"  width="40.0%" />
</div>

| IMU Connector（fischer 102） | USB Cable/SH1.0-4P Connector |
| :------------: | :------------: |
| Pin 1 | USB GND |
| Pin 8 | USB DP(D+) |
| Pin 9 | USB DM(D-) |
| Pin 4 | USB Vcc |
| Pin 1 | SH1.0-4P Pin1 |
| Pin 5 | SH1.0-4P Pin2 |
| Pin 6 | SH1.0-4P Pin3 |
| Pin 7 | SH1.0-4P Pin4 |

#### Camera  Cable

<div align="center">
<img src="./pics/Camera_hardware.png"  width="80.0%" />
</div>
| Hikvision Trigger Cable | SH1.0-2P Connector |
| :------------: | :------------: |
| GND | Pin1 |
| GPIO | Pin2 |

#### IMU-Lidar  Cable

SH1.0-6P cable 10cm

### 3.4 HandHold
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

|Component|Pics|Num|
| :------------: | :------------: | -------------- |
|Mounting Plate(3mm plate)|<img src="./pics/board.png" width=50%  />| 1 |
|support(3D printing)|<img src="./pics/support.png" width=50%  />| 1 |
|4S 2200mAh Li-Battery(XT60)| <img src="./pics/battery.jpeg" width=50%  /> | 1 |
| XT60 to DC 5.5 - 2.1 Cable | <img src="./pics/xt60-dc.jpeg" width=50%  /> | 1 |
| M3x6 D8xH8 Rubber Shock Absorber | <img src="./pics/rubber.jpeg" width=50%  /> | 4 |
| Handheld Tripod | <img src="./pics/tp97.jpeg" width=50%  /> | 1 |

## 4. How To Run
### 4.1 Set Drivers
#### IMU Driver
<div align="center">
<img src="./pics/IMU_settings1.png"  width="80.0%" />
</div>
Enable the Sample Time Fine and Sample Time Coarse options.

<div align="center">
<img src="./pics/IMU_settings2.png"  width="80.0%" />
</div>
Add the Start Sampling function in Synchronization, and set the Polarity to Rising Edge.

<div align="center">
<img src="./pics/IMU_settings3.png"  width="80.0%" />
</div>
Add the Sync Out function in Synchronization, set the Skip Factor to 399, and the Pulse Width to 100,000 µs.

#### Lidar Driver
<div align="center">
<img src="./pics/Lidar_settings1.png"  width="80.0%" />
</div>
Turn on Pulse Trigger Switch in Robosense Driver, and set the Pulse Start Angle to 342°, set the Pulse Width to 10,000,000 ns.
#### Camera Driver

The camera can be directly configured using the ROS driver without the need for prior setup.


### 4.2 Run ROS Drivers

Ensure that all sensors are connected first, all drivers are running, and only then should power be supplied to the LiDAR.

Before running the drivers, you must configure the hardware-related parameters in the `src/mvs_ros_pkg/config/camera_trigger.yaml`, `src/rslidar_sdk/config/config.yaml`, and `src/xsens_ros_mti_driver/param/xsens_mti_node.yaml` files, such as the device `SerialNumber`, `msop_port`, etc.

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
