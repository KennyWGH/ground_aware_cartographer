/*
 * Copyright 2021 Guanhua WANG
 */

#ifndef EFFICIENT_ONLINE_SEGMENTATION_H_
#define EFFICIENT_ONLINE_SEGMENTATION_H_

#include <vector>
#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cartographer_ros/fast_ground_detection/detection_utility.h"
#include "cartographer_ros/fast_ground_detection/smart_sector.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer_ros {

class FastGroundDetection {
  public:
    FastGroundDetection(const DetectionParams& params = DetectionParams());
    ~FastGroundDetection();

    FastGroundDetection(const FastGroundDetection&) = delete;
    FastGroundDetection& operator=(const FastGroundDetection&) = delete;

    void ResetParameters(const DetectionParams& params);

    void Segment(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_data, 
                std::vector<int>* labels_out,
                float& ground_evaluation, 
                bool use_intensity=false);

    void Segment(pcl::PointCloud<PointXYZIRT>::Ptr& cloud_data, 
                std::vector<int>* labels_out, 
                float& ground_evaluation, 
                bool use_intensity=false);

    void Segment(::cartographer::sensor::PointCloudWithIntensities& cloud_data, 
                std::vector<int>* labels_out,
                float& ground_evaluation, 
                bool use_intensity=false);

    pcl::PointCloud<pcl::PointXYZI>::Ptr
    GetTransformedCommonCloud();
    pcl::PointCloud<PointXYZIRT>::Ptr
    GetTransformedCustomCloud();

    std::vector<BasicLine>& GetExtractedLines();

  private:
    float EvaluateGroundScore();

  private:
    struct SectorInfo
    {
      int is_ground = 0;
      bool counted = false;

      SectorInfo():is_ground(0),counted(false) {}

      void Reset()
      {
        is_ground = 0;
        counted = false;
      }
    };
    
    DetectionParams params_;
    std::vector<SmartSector> sectors_;
    std::vector<SectorInfo> ground_distribution_;
    Eigen::Affine3f sensor_pose_in_base_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr common_cloud_in_base_;
    pcl::PointCloud<PointXYZIRT>::Ptr custom_cloud_in_base_;
    bool ordinary_cloud_used = false;
    bool custom_cloud_used = false;
    std::vector<BasicLine> extracted_lines_;

    std::uint64_t num_received_msgs = 0;
    double accumulated_run_time = 0;
    float kGroundIntensity = 5;
    float kIntensityIncre = 20; // intensity lower bound of non-ground points.
    bool kPrintLog = true;

};

}  // namespace cartographer_ros

#endif // EFFICIENT_ONLINE_SEGMENTATION_H_
