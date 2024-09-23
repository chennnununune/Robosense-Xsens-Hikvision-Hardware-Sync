#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <chrono>
#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/stat.h>

struct time_stamp {
  int64_t high;
  int64_t low;
};

time_stamp *pointt;

using namespace std;
// 用的是这份代码
unsigned int g_nPayloadSize = 0;
bool is_undistorted = true;
bool exit_flag = false;
image_transport::Publisher pub;
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
std::string CameraName;

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }

  // 从config中获取参数
  int ExposureAuto = Params["ExposureAuto"];
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int ExposureTime = Params["ExposureTime"];
  int GainAuto = Params["GainAuto"];
  float Gain = Params["Gain"];
  float Gamma = Params["Gamma"];
  int GammaSlector = Params["GammaSelector"];
  int ReverseX = Params["ReverseX"];
  int ReverseY = Params["ReverseY"];
  int Binning = Params["Binning"];

  int nRet;

  // 如果是自动曝光
  if (ExposureAuto == 2) {
    // 设置ExposureAuto
    nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", ExposureAuto);
    if (MV_OK == nRet) {
      std::string msg = "Set Exposure Auto: " + ExposureAutoStr[ExposureAuto];
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure mode");
    }

    // 设置最短曝光时间
    nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) +
          "ms";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time Lower");
    }

    // 设置最长曝光时间
    nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) +
          "ms";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time Upper");
    }
  }

  // 如果是固定曝光
  if (ExposureAuto == 0) {
    // 设置ExposureAuto
    nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", ExposureAuto);
    if (MV_OK == nRet) {
      std::string msg = "Set Exposure Auto: " + ExposureAutoStr[ExposureAuto];
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure mode");
    }

    // 设置曝光时间
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time: " + std::to_string(ExposureTime) + "ms";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time");
    }
  }

  // 设置自动gain模式
  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);
  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Gain auto mode");
  }

  // 若非自动gain
  if (GainAuto == 0) {
    nRet = MV_CC_SetGain(handle, Gain);
    if (MV_OK == nRet) {
      std::string msg = "Set Gain: " + std::to_string(Gain);
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Gain");
    }
  }

  // 设置gamma模式
  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  if (MV_OK == nRet) {
    std::string msg = "Set GammaSlector: " + GammaSlectorStr[GammaSlector];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set GammaSlector");
  }

  // 设置gamma值
  nRet = MV_CC_SetGamma(handle, Gamma);
  if (MV_OK == nRet) {
    std::string msg = "Set Gamma: " + std::to_string(Gamma);
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Gamma");
  }

  // 设置x翻转
  nRet = MV_CC_SetBoolValue(handle, "ReverseX", ReverseX);
  if (MV_OK == nRet) {
    std::string msg = "Set ReverseX: " + std::to_string(ReverseX);
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set ReverseX");
  }

  // 设置y翻转
  nRet = MV_CC_SetBoolValue(handle, "ReverseY", ReverseY);
  if (MV_OK == nRet) {
    std::string msg = "Set ReverseY: " + std::to_string(ReverseY);
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set ReverseY");
  }

  // 设置Binning
  nRet = MV_CC_SetEnumValue(handle, "BinningHorizontal", Binning);
  if (MV_OK == nRet) {
    std::string msg = "Set BinningHorizontal: " + std::to_string(Binning);
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set BinningHorizontal");
  }

  nRet = MV_CC_SetEnumValue(handle, "BinningVertical", Binning);
  if (MV_OK == nRet) {
    std::string msg = "Set BinningVertical: " + std::to_string(Binning);
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set BinningVertical");
  }
}

void PressEnterToExit(void) {
  int c;
  while ((c = getchar()) != '\n' && c != EOF)
    ;
  fprintf(stderr, "\nPress enter to exit.\n");
  while (getchar() != '\n')
    ;
  exit_flag = true;
  sleep(1);
}

// 线程获取
static void *WorkThread(void *pUser) {
  int nRet = MV_OK;

  // ch:获取数据包大小 | en:Get payload size
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    return NULL;
  }

  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  unsigned char *pData =
      (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
  if (NULL == pData)
    return NULL;

  unsigned int nDataSize = stParam.nCurValue;

  while (ros::ok()) {
    nRet =
        MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
    if (nRet == MV_OK) {
      ros::Time time_pc = ros::Time::now();
      cv::Mat srcImage;
      srcImage =
          cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      // printf("a = %ld b = %ld\n", , pointt->low);
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(std_msgs::Header(), "rgb8", srcImage).toImageMsg();
      // 等待图像对应雷达帧并赋予时间戳
      bool discard_flag = false;
      ros::NodeHandle nh_;
      ros::Rate r(100); // 10ms为周期查询
      while(ros::ok())
      {
        ros::Time current_pc_time = ros::Time::now();
        ros::Duration image_to_current_pc_time_bias = current_pc_time - time_pc;
        if(image_to_current_pc_time_bias.toSec() > 0.075) // 若等待时间超过0.075秒，则舍弃这一帧图像
        {
          discard_flag = true;
          break;
        }

        std::string last_lidar_data_pc_time_string;
        double last_lidar_data_pc_time;
        nh_.getParam("last_lidar_data_pc_time",last_lidar_data_pc_time_string);
        last_lidar_data_pc_time = std::stod(last_lidar_data_pc_time_string);
        double image_to_lidar_bias = last_lidar_data_pc_time - time_pc.toSec();

        if(image_to_lidar_bias >= 0.0 && image_to_lidar_bias <= 0.1)  // 最近雷达帧获取时间处于该图像获取时间的0到100ms内，为同一时间获取，为图像赋予雷达的时间戳
        {
          std::string last_lidar_data_stamp_secs_string;
          std::string last_lidar_data_stamp_nsecs_string;
          unsigned int last_lidar_data_stamp_secs;
          unsigned int last_lidar_data_stamp_nsecs;
          nh_.getParam("last_lidar_data_stamp_secs",last_lidar_data_stamp_secs_string);
          nh_.getParam("last_lidar_data_stamp_nsecs",last_lidar_data_stamp_nsecs_string);
          last_lidar_data_stamp_secs = std::stoul(last_lidar_data_stamp_secs_string);
          last_lidar_data_stamp_nsecs = std::stoul(last_lidar_data_stamp_nsecs_string);
          ros::Time last_lidar_data_stamp(last_lidar_data_stamp_secs,last_lidar_data_stamp_nsecs);
          msg->header.stamp = last_lidar_data_stamp;
          break;
        }
        r.sleep();
      }

      if(discard_flag)
      {
        ROS_INFO_STREAM("Image Discarded");
        continue;
      }

      //msg->header.stamp = ros::Time::now();
      pub.publish(msg);

      std::string debug_msg;
      debug_msg = CameraName + " GetOneFrame,nFrameNum[" +
                  std::to_string(stImageInfo.nFrameNum) + "], Sync success";
      ROS_INFO_STREAM(debug_msg.c_str());
    }

    if (exit_flag)
      break;
  }

  if (pData) {
    free(pData);
    pData = NULL;
  }

  return 0;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "mvs_trigger");
  std::string params_file = std::string(argv[1]);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  int nRet = MV_OK;
  void *handle = NULL;
  ros::Rate loop_rate(10);
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];
  std::string camera_name = Params["CameraName"];
  CameraName = camera_name;
  pub = it.advertise(pub_topic, 1);
  const char *user_name = getlogin();
  std::string path_for_time_stamp = "/home/" + std::string(user_name) + "/timeshare";
  const char *shared_file_name = path_for_time_stamp.c_str();

  int fd = open(shared_file_name, O_RDWR);

  // pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
  //                             MAP_SHARED, fd, 0);
  while (ros::ok()) {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    // 枚举检测到的相机数量
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
      printf("Enum Devices fail!");
      break;
    }

    bool find_expect_camera = false;
    int expect_camera_index = 0;
    if (stDeviceList.nDeviceNum == 0) {
      ROS_ERROR_STREAM("No Camera.");
      continue;
    } else {
      // 根据serial number启动指定相机
      for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        std::string serial_number =
            std::string((char *)stDeviceList.pDeviceInfo[i]
                            ->SpecialInfo.stUsb3VInfo.chSerialNumber);
        if (expect_serial_number == serial_number) {
          find_expect_camera = true;
          expect_camera_index = i;
          break;
        }
      }
    }
    if (!find_expect_camera) {
      std::string msg =
          "Can not find the camera with serial number " + expect_serial_number;
      ROS_ERROR_STREAM(msg.c_str());
      break;
    }

    nRet = MV_CC_CreateHandle(&handle,
                              stDeviceList.pDeviceInfo[expect_camera_index]);
    if (MV_OK != nRet) {
      ROS_ERROR_STREAM("Create Handle fail");
      break;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
      printf("Open Device fail\n");
      break;
    }

    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
    if (MV_OK != nRet) {
      printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
      break;
    }

    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet) {
      printf("Get PayloadSize fail\n");
      break;
    }
    g_nPayloadSize = stParam.nCurValue;

    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);
    if (nRet != MV_OK) {
      printf("Pixel setting can't work.");
      break;
    }

    setParams(handle, params_file);

    // 设置触发模式为on
    // set trigger mode as on
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);
    if (MV_OK != nRet) {
      printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
      break;
    }

    // 设置触发源
    // set trigger source
    nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE2);
    if (MV_OK != nRet) {
      printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
      break;
    }

    // 设置输入防抖时间为0
    nRet = MV_CC_SetIntValue(handle, "LineDebouncerTime", 0);
    if (MV_OK == nRet) {
      std::string msg = "Set LineDebouncerTime: " + std::to_string(0);
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set LineDebouncerTime");
    }

    // 设置触发缓存模式为无缓存
    nRet = MV_CC_SetBoolValue(handle, "TriggerCacheEnable", false);
    if (MV_OK == nRet) {
      std::string msg = "Set TriggerCacheEnable: " + std::to_string(false);
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set TriggerCacheEnable");
    }

    ROS_INFO("Finish all params set! Start grabbing...");
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
      printf("Start Grabbing fail.\n");
      break;
    }

    pthread_t nThreadID;
    nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
    if (nRet != 0) {
      printf("thread create failed.ret = %d\n", nRet);
      break;
    }

    PressEnterToExit();

    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
      break;
    }

    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
      break;
    }

    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
      break;
    }

    break;
  }
  // munmap(pointt, sizeof(time_stamp) * 5);
  return 0;
}