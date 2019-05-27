/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "pandora/pandora.h"
#include "src/pandora_camera.h"
#include "src/pandora_client.h"
#include "src/tcp_command_client.h"
#include "yaml-cpp/yaml.h"

namespace apollo {
namespace drivers {
namespace hesai {

#define PANDORA_TCP_COMMAND_PORT (9347)

class Pandora_Internal {
 public:
  Pandora_Internal(
      std::string device_ip, const uint16_t lidar_port, const uint16_t gps_port,
      boost::function<void(boost::shared_ptr<PPointCloud>, double)>
          pcl_callback,
      boost::function<void(double)> gps_callback, uint16_t start_angle,
      const uint16_t pandoraCameraPort,
      boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp,
                           int picid, bool distortion)>
          cameraCallback,
      bool enable_camera, int tz, std::string frame_id);
  ~Pandora_Internal();
  int LoadLidarCorrectionFile(const std::string &correction_content);
  void ResetLidarStartAngle(uint16_t start_angle);
  int UploadCameraCalibrationFile(const CameraCalibration calibs[5]);
  int GetCameraCalibration(CameraCalibration calibs[5]);
  int ResetCameraClibration();
  void GetCalibrationFromDevice();
  int ParseCameraCalibration(std::string contents,
                             CameraCalibration calibs[5]);
  int GenerateCameraCalibration(const CameraCalibration calibs[5],
                                std::string *contents);
  int Start();
  void Stop();

 private:
  Pandar40P *pandar40p_;
  PandoraCamera *pandora_camera_;
  void *tcp_command_client_;
  boost::thread *get_calibration_thr_;
  bool enable_get_calibration_thr_;

  bool got_lidar_calibration_;
  bool got_camera_calibration_;

  CameraCalibration camera_calibs_[5];

  bool enable_camera_;
};

Pandora_Internal::Pandora_Internal(
    std::string device_ip, const uint16_t lidar_port, const uint16_t gps_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)> pcl_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle,
    const uint16_t pandoraCameraPort,
    boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp,
                         int picid, bool distortion)>
        cameraCallback,
    bool enable_camera, int tz, std::string frame_id) {
  enable_camera_ = enable_camera;
  pandora_camera_ = NULL;
  pandar40p_ = NULL;

  pandar40p_ = new Pandar40P(device_ip, lidar_port, gps_port, pcl_callback,
                             gps_callback, start_angle, tz, frame_id);

  if (enable_camera_) {
    pandora_camera_ = new PandoraCamera(device_ip, pandoraCameraPort,
                                        cameraCallback, NULL, tz);
  }

  tcp_command_client_ =
      TcpCommandClientNew(device_ip.c_str(), PANDORA_TCP_COMMAND_PORT);
  if (!tcp_command_client_) {
    std::cout << "Init TCP Command Client Failed" << std::endl;
  }
  get_calibration_thr_ = NULL;
  enable_get_calibration_thr_ = false;

  got_lidar_calibration_ = false;
  got_camera_calibration_ = false;
}

Pandora_Internal::~Pandora_Internal() {
  Stop();
  if (pandar40p_) {
    delete pandar40p_;
  }

  if (pandora_camera_) {
    delete pandora_camera_;
  }
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int Pandora_Internal::LoadLidarCorrectionFile(const std::string &correction_content) {
  return pandar40p_->LoadCorrectionFile(correction_content);
}

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void Pandora_Internal::ResetLidarStartAngle(uint16_t start_angle) {
  if (!pandar40p_) return;
  pandar40p_->ResetStartAngle(start_angle);
}

int Pandora_Internal::Start() {
  Stop();
  if (pandora_camera_) {
    pandora_camera_->Start();
  }

  if (pandar40p_) {
    pandar40p_->Start();
  }

  enable_get_calibration_thr_ = true;
  get_calibration_thr_ = new boost::thread(
      boost::bind(&Pandora_Internal::GetCalibrationFromDevice, this));
}

void Pandora_Internal::Stop() {
  if (pandar40p_) pandar40p_->Stop();
  if (pandora_camera_) pandora_camera_->Stop();

  enable_get_calibration_thr_ = false;
  if (get_calibration_thr_) {
    get_calibration_thr_->join();
  }
}

int Pandora_Internal::UploadCameraCalibrationFile(
    const CameraCalibration calibs[5]) {
  std::string contents;
  int ret = 0;
  ret = GenerateCameraCalibration(calibs, &contents);
  if (ret != 0) {
    std::cout << "Generate Camera Calibration Failed" << std::endl;
    return ret;
  }

  return TcpCommandSetCalibration(tcp_command_client_, contents.c_str(),
                                  contents.size());
}

int Pandora_Internal::GetCameraCalibration(CameraCalibration calibs[5]) {
  if (!got_camera_calibration_) {
    return -1;
  }

  for (int i = 0; i < 5; ++i) {
    /* assign */
    calibs[i].cameraK = camera_calibs_[i].cameraK;
    calibs[i].cameraD = camera_calibs_[i].cameraD;
    calibs[i].cameraT = camera_calibs_[i].cameraT;
    calibs[i].cameraR = camera_calibs_[i].cameraR;
  }
}

int Pandora_Internal::ResetCameraClibration() {
  if (!tcp_command_client_) {
    std::cout << "Pandora Tcp Command Client is NULL" << std::endl;
    return -1;
  }
  return TcpCommandResetCalibration(tcp_command_client_);
}

void Pandora_Internal::GetCalibrationFromDevice() {
  if (!tcp_command_client_) {
    return;
  }
  int32_t ret = 0;
  while (enable_get_calibration_thr_ && !got_lidar_calibration_ &&
         !got_camera_calibration_) {
    if (!got_lidar_calibration_) {
      // get lidar calibration.
      char *buffer = NULL;
      uint32_t len = 0;

      ret = TcpCommandGetLidarCalibration(tcp_command_client_, &buffer, &len);
      if (ret == 0 && buffer) {
        // success;
        got_lidar_calibration_ = true;
        if (pandar40p_) {
          ret = pandar40p_->LoadCorrectionFile(std::string(buffer));
          if (ret != 0) {
            std::cout << "Parse Lidar Correction Error" << std::endl;
            got_lidar_calibration_ = false;
          } else {
            std::cout << "Parse Lidar Correction Success!!!" << std::endl;
          }
        }
        free(buffer);
      }
    }

    // If got lidar's calibration , and camera is disabled
    // stop to get camera's calibration
    if (!enable_camera_ && got_lidar_calibration_) break;

    if (!got_camera_calibration_) {
      // get camera calibration.
      char *buffer = NULL;
      uint32_t len = 0;

      ret = TcpCommandGetCalibration(tcp_command_client_, &buffer, &len);
      if (ret == 0 && buffer) {
        // success;
        ret = ParseCameraCalibration(std::string(buffer), camera_calibs_);
        if (ret == 0) {
          got_camera_calibration_ = true;
          std::cout << "Parse Camera Calibration Success!!!" << std::endl;
        } else {
          std::cout << "Parse Camera Calibration Error" << std::endl;
          free(buffer);
          continue;
        }

        std::vector<cv::Mat> cameras_k;
        std::vector<cv::Mat> cameras_d;

        for (int i = 0; i < 5; ++i) {
          cameras_k.push_back(camera_calibs_[i].cameraK);
          cameras_d.push_back(camera_calibs_[i].cameraD);
        }
        if (pandora_camera_) {
          pandora_camera_->loadIntrinsics(cameras_k, cameras_d);
        }

        free(buffer);
        std::cout << "TcpCommandGetCalibration Success" << std::endl;
      }
    }

    sleep(1);
  }
}

int Pandora_Internal::ParseCameraCalibration(std::string contents,
                                             CameraCalibration calibs[5]) {
  std::cout << "Parse Camera Calibration..." << std::endl;
  std::cout << contents << std::endl;
  contents = "4:\n\
  K: [454.7254149884122, 0, 648.0782453544582, 0, 450.8323936612145, 364.8662353720151, 0, 0, 1]\n\
  D: [-0.09614886709176985, -0.007151356844960435, -0.0005269301651919918, 2.670521453365162e-05, 0.00556467662152453]\n\
  Extrinsic:\n\
    rotation:\n\
      y: 0.4869359353389395\n\
      z: -0.4894198621682698\n\
      x: -0.5146420355461454\n\
      w: 0.5084340356818216\n\
    translation:\n\
      y: 0.03085358586539645\n\
      z: -0.06350385121143094\n\
      x: 0.0462138588404069\n\
3:\n\
  K: [456.150051137236, 0, 653.1394664443974, 0, 451.9998249436122, 356.0839015730878, 0, 0, 1]\n\
  D: [-0.0940098922466583, -0.01153260847242368, -0.000524385055558979, -0.0002787212215726739, 0.007886068293924367]\n\
  Extrinsic:\n\
    rotation:\n\
      y: -0.02252435350813502\n\
      z: 0.01570687834653467\n\
      x: -0.7076140811346469\n\
      w: 0.7060653366733259\n\
    translation:\n\
      y: 0.0716031579042775\n\
      z: -0.05708226592104329\n\
      x: -0.007542048964797545\n\
2:\n\
  K: [456.0961775929016, 0, 649.6938714465472, 0, 452.1903853765498, 365.8819282687128, 0, 0, 1]\n\
  D: [-0.0946285039575155, -0.00916000529998711, 0.0002090328280001688, -0.0009871734857353641, 0.005825706567851142]\n\
  Extrinsic:\n\
    rotation:\n\
      y: 0.5082465249778154\n\
      z: -0.5148129588585233\n\
      x: 0.4922277164356043\n\
      w: -0.4841125513885148\n\
    translation:\n\
      y: -0.02491800196583054\n\
      z: -0.06817869373582143\n\
      x: -0.08429583685362887\n\
1:\n\
  K: [455.7047284284406, 0, 647.9473133566629, 0, 451.5286392601823, 354.2331813445593, 0, 0, 1]\n\
  D: [-0.09642059624970495, -0.008495630603823134, -0.0001492444428318642, -0.0002679739263773861, 0.006613593953191534]\n\
  Extrinsic:\n\
    rotation:\n\
      y: -0.708655277299056\n\
      z: 0.7052101006782753\n\
      x: 0.01224058132813469\n\
      w: -0.01834611747167085\n\
    translation:\n\
      y: -0.06687954346056678\n\
      z: -0.05451981257919083\n\
      x: 0.01850521539088982\n\
0:\n\
  K: [1461.317641253868, 0, 650.5533608051251, 0, 1447.137849649878, 355.3033160801929, 0, 0, 1]\n\
  D: [-0.09642059624970495, -0.008495630603823134, -0.0001492444428318642, -0.0002679739263773861, 0.006613593953191534]\n\
  Extrinsic:\n\
    rotation:\n\
      y: -0.708655277299056\n\
      z: 0.7052101006782753\n\
      x: 0.01224058132813469\n\
      w: -0.01834611747167085\n\
    translation:\n\
      y: -0.06687954346056678\n\
      z: -0.05451981257919083\n\
      x: 0.01850521539088982";
  if (contents.empty()) {
    std::cout << "string is empty" << std::endl;
    return -1;
  }
  YAML::Node yn = YAML::Load(contents);
  std::string cameraId;
  for (int id = 0; id < 5; ++id) {
    // cameraId = std::to_string(i);
    cameraId = boost::lexical_cast<std::string>(id);
    cv::Mat intrinsicK, intrinsicD;
    // get intrinsicK
    if (yn[cameraId]["K"].IsDefined()) {
      intrinsicK = cv::Mat::zeros(3, 3, CV_64FC1);
      for (int i = 0; i < yn[cameraId]["K"].size(); ++i) {
        intrinsicK.at<double>(i) = yn[cameraId]["K"][i].as<double>();
      }
      calibs[id].cameraK = intrinsicK;
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
    // GET intrinsicD
    if (yn[cameraId]["D"].IsDefined()) {
      // std::cout<<"type: " << yn[cameraId]["D"].Type()<<std::endl;
      intrinsicD = cv::Mat::zeros(yn[cameraId]["D"].size(), 1, CV_64FC1);
      for (int i = 0; i < yn[cameraId]["D"].size(); ++i) {
        intrinsicD.at<double>(i) = yn[cameraId]["D"][i].as<double>();
      }
      calibs[id].cameraD = intrinsicD;
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
    // get camera
    if (yn[cameraId]["Extrinsic"]["rotation"]["x"].IsDefined() &&
        yn[cameraId]["Extrinsic"]["rotation"]["y"].IsDefined() &&
        yn[cameraId]["Extrinsic"]["rotation"]["z"].IsDefined() &&
        yn[cameraId]["Extrinsic"]["rotation"]["w"].IsDefined()) {
      calibs[id].cameraR.push_back(
          yn[cameraId]["Extrinsic"]["rotation"]["w"].as<double>());
      calibs[id].cameraR.push_back(
          yn[cameraId]["Extrinsic"]["rotation"]["x"].as<double>());
      calibs[id].cameraR.push_back(
          yn[cameraId]["Extrinsic"]["rotation"]["y"].as<double>());
      calibs[id].cameraR.push_back(
          yn[cameraId]["Extrinsic"]["rotation"]["z"].as<double>());
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
    // get cameraR
    if (yn[cameraId]["Extrinsic"]["translation"]["x"].IsDefined() &&
        yn[cameraId]["Extrinsic"]["translation"]["y"].IsDefined() &&
        yn[cameraId]["Extrinsic"]["translation"]["z"].IsDefined()) {
      calibs[id].cameraT.push_back(
          yn[cameraId]["Extrinsic"]["translation"]["x"].as<double>());
      calibs[id].cameraT.push_back(
          yn[cameraId]["Extrinsic"]["translation"]["y"].as<double>());
      calibs[id].cameraT.push_back(
          yn[cameraId]["Extrinsic"]["translation"]["z"].as<double>());
    } else {
      printf("invalid intrinsicFile content\n");
      return -1;
    }
  }
  return 0;
}

int Pandora_Internal::GenerateCameraCalibration(
    const CameraCalibration calibs[5], std::string *contents) {
  std::string cameraId;
  YAML::Node node;
  for (int id = 0; id < 5; ++id) {
    YAML::Node nodeK, nodeD, nodeT, nodeR;
    // nodeK.SetStyle(YAML::EmitterStyle::Flow);
    // nodeD.SetStyle(YAML::EmitterStyle::Flow);
    // nodeT.SetStyle(YAML::EmitterStyle::Flow);
    // nodeR.SetStyle(YAML::EmitterStyle::Flow);
    std::string id_str = boost::lexical_cast<std::string>(id);
    int rowsK = calibs[id].cameraK.rows;
    int colsK = calibs[id].cameraK.cols;
    int rowsD = calibs[id].cameraD.rows;
    int colsD = calibs[id].cameraD.cols;

    for (int i = 0; i < rowsK * colsK; i++) {
      nodeK[i] = calibs[id].cameraK.at<double>(i);
    }
    node[id_str]["K"] = nodeK;

    for (int i = 0; i < rowsD * colsD; i++) {
      nodeD[i] = calibs[id].cameraD.at<double>(i);
    }
    node[id_str]["D"] = nodeD;

    for (int i = 0; i < calibs[id].cameraT.size(); i++) {
      nodeT[i] = boost::lexical_cast<std::string>(calibs[id].cameraT[i]);

      node[id_str]["Extrinsic"]["rotation"]["w"] = calibs[id].cameraR[0];
      node[id_str]["Extrinsic"]["rotation"]["x"] = calibs[id].cameraR[1];
      node[id_str]["Extrinsic"]["rotation"]["y"] = calibs[id].cameraR[2];
      node[id_str]["Extrinsic"]["rotation"]["z"] = calibs[id].cameraR[3];

      node[id_str]["Extrinsic"]["translation"]["x"] = calibs[id].cameraT[0];
      node[id_str]["Extrinsic"]["translation"]["y"] = calibs[id].cameraT[1];
      node[id_str]["Extrinsic"]["translation"]["z"] = calibs[id].cameraT[2];
    }
  }
  *contents = YAML::Dump(node);
  // std::cout<<node<<std::endl;
  return 0;
}

/*****************************************************************************************
Pandora Part
*****************************************************************************************/
/**
 * @brief Constructor
 * @param device_ip         The ip of the device
 *        lidar_port        The port number of lidar data
 *        gps_port          The port number of gps data
 *        pcl_callback      The callback of PCL data structure
 *        gps_callback      The callback of GPS structure
 *        start_angle       The start angle of every point cloud
 *        pandoraCameraPort The port of camera data
 *        cameraCallback    the call back for camera data
 */
Pandora::Pandora(
    const std::string &device_ip, const uint16_t lidar_port, const uint16_t gps_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)> pcl_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle,
    const uint16_t pandoraCameraPort,
    boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp,
                         int picid, bool distortion)>
        cameraCallback,
    bool enable_camera, int tz, std::string frame_id) {
  internal_ = new Pandora_Internal(
      device_ip, lidar_port, gps_port, pcl_callback, gps_callback, start_angle,
      pandoraCameraPort, cameraCallback, enable_camera, tz, frame_id);
}

/**
 * @brief deconstructor
 */
Pandora::~Pandora() { delete internal_; }

/**
 * @brief load the lidar correction file
 * @param contents The correction contents of lidar correction
 */
int Pandora::LoadLidarCorrectionFile(const std::string &correction_content) {
  internal_->LoadLidarCorrectionFile(correction_content);
}

/**
 * @brief Reset Lidar's start angle.
 * @param angle The start angle
 */
void Pandora::ResetLidarStartAngle(uint16_t start_angle) {
  internal_->ResetLidarStartAngle(start_angle);
}

/**
 * @brief Upload the camera calibration contents to Pandora Device.
 * @param calibs calibration contents , include camera intrinsics and
 * extrinsics.
 */
int Pandora::UploadCameraCalibration(const CameraCalibration calibs[5]) {
  internal_->UploadCameraCalibrationFile(calibs);
}

/**
 * @brief Get Camera's Calibration, include camera intrinsics and extrinsics.
 * @param calibs calibration contents , include camera intrinsics and
 * extrinsics.
 */
int Pandora::GetCameraCalibration(CameraCalibration calibs[5]) {
  internal_->GetCameraCalibration(calibs);
}

/**
 * @brief Reset camera calibration as factory-set.
 */
int Pandora::ResetCameraClibration() { internal_->ResetCameraClibration(); }

/**
 * @brief Run SDK.
 */
int Pandora::Start() { internal_->Start(); }

/**
 * @brief Stop SDK.
 */
void Pandora::Stop() { internal_->Stop(); }

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
