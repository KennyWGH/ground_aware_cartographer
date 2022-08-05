/*
 * Copyright 2021 Guanhua WANG
 */

#include "cartographer_ros/fast_ground_detection/fast_ground_detection.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <pcl/common/transforms.h>

namespace cartographer_ros {

FastGroundDetection::FastGroundDetection(const DetectionParams& params)
{
    ResetParameters(params);
    common_cloud_in_base_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    custom_cloud_in_base_.reset(new pcl::PointCloud<PointXYZIRT>());
}

FastGroundDetection::~FastGroundDetection() {}

void FastGroundDetection::ResetParameters(const DetectionParams& params)
{
    params_ = params;
    sectors_.clear();
    sectors_.resize(params_.kNumSectors);
    for (std::size_t i=0; i<sectors_.size(); i++) {
        sectors_[i] = SmartSector(i/*sector id*/,
                                params_.kLidarRows,
                                0, // params_.kSensorHeight,
                                params_.kGroundSameLineTolerance,
                                params_.kGroundSlopeTolerance,
                                params_.kGroundYInterceptTolerance,
                                params_.kGroundPointLineDistThres);
    }
    ground_distribution_.resize(params_.kNumSectors);
    std::fill(ground_distribution_.begin(), ground_distribution_.end(), SectorInfo());
    sensor_pose_in_base_ = params_.kBaseToSensor;
    std::cout << "sensor_pose_in_base_: " << std::endl 
        << sensor_pose_in_base_.matrix() << std::endl 
        << " " << std::endl;
}

// implementation 1.
void FastGroundDetection::Segment(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_data, 
            std::vector<int>* labels_out, float& ground_evaluation, bool use_intensity) 
{
    ordinary_cloud_used = true;
    custom_cloud_used = false;
    num_received_msgs++;
    if (kPrintLog) {
        std::cout << "Segmenting cloud with " << cloud_data->size() << " points... -------------- "
                  << "[msg #" << num_received_msgs << "] " << std::endl;
    }

    clock_t time_start = clock();

    // step01 register every and each point into corresponding sector!
    for (auto& sector : sectors_) {sector.Reset();}
    pcl::transformPointCloud(*cloud_data, *common_cloud_in_base_, sensor_pose_in_base_);
    for (std::size_t i=0; i<cloud_data->size(); ++i)
    {
        // if (use_intensity) {cloud_data->points[i].intensity += kIntensityIncre;}
        if (use_intensity) {cloud_data->points[i].intensity = 10;}
        if (common_cloud_in_base_->points[i].z > 1.0) continue;
        float ang_azimuth = std::atan2(cloud_data->points[i].y, cloud_data->points[i].x);
        while (ang_azimuth < 0) { ang_azimuth += 2*M_PI; } /* from [-pi,+pi] to [0, 2pi] */
        float xy_range = std::sqrt( cloud_data->points[i].x*cloud_data->points[i].x
                                    +cloud_data->points[i].y*cloud_data->points[i].y );
        float ang_elevation = std::atan2(cloud_data->points[i].z, xy_range); /* naturally [-pi,+pi] */
        if (ang_elevation<params_.kLidarVertAngMin || ang_elevation>params_.kLidarVertAngMax) {
            continue;
        }

        int rowIdx=0, colIdx=0;
        rowIdx = round((ang_elevation - params_.kLidarVertFovMin) * params_.kLidarVertResInv);
        colIdx = floor(ang_azimuth*params_.kLidarHorizResInv);
        if (rowIdx < 0 || rowIdx >= params_.kLidarRows 
            || colIdx < 0 || colIdx >= params_.kLidarCols) {
            continue;
        }
        int sectorIdx = floor(colIdx/params_.kColsPerSector/*5*/);
        if (sectorIdx<0 || sectorIdx>=params_.kNumSectors) {
            continue;
        }

        // Note that here we must pass in point in base (or ground) frame!
        // Remember that the `intensity` represents x-y range!!!!
        pcl::PointXYZI point3D = common_cloud_in_base_->points[i];
        point3D.intensity = std::sqrt(point3D.x*point3D.x
                                     +point3D.y*point3D.y);
        sectors_[sectorIdx].AddPoint(point3D, i/*original index*/, rowIdx/*bin id*/);
    }
    clock_t time_stop01 = clock();

    // step02 extract line within each sector.
    extracted_lines_.clear();
    std::vector<int> ground_indices;
    for (std::size_t i=0; i<sectors_.size(); ++i) {
        sectors_[i].RunLineExtraction();
        auto& result = sectors_[i].GetGroundPointIndices(); 
        ground_indices.insert(ground_indices.end(), result.begin(), result.end());
        auto& geometry_lines = sectors_[i].GetExtractedLines();
        extracted_lines_.insert(extracted_lines_.end(), geometry_lines.begin(), geometry_lines.end());
        ground_distribution_[i].Reset();
        if (!result.empty()) {
            ground_distribution_[i].is_ground = 1;
        }
    }

    // step03 evaluate ground distribution score.
    ground_evaluation = EvaluateGroundScore();

    // step04 project labels to original `cloud_data`.
    labels_out->clear();
    labels_out->resize(cloud_data->size());
    std::fill(labels_out->begin(), labels_out->end(), 0); /* 0(NonGround), 1(Ground), 2(Others) */
    for (const auto& index : ground_indices) {
        (*labels_out)[index] = 1;
        if (use_intensity) {cloud_data->points[index].intensity=100;}
    }
    if (use_intensity) { /* expand intensity interval to [0, 255]. */
        cloud_data->points[0].intensity=0;
        cloud_data->points[1].intensity=255;
    } 

    clock_t time_end = clock();
    float timePhase1 = (float(time_stop01-time_start))/CLOCKS_PER_SEC;
    float timeTotal = (float(time_end-time_start))/CLOCKS_PER_SEC;
    accumulated_run_time += timeTotal;
    if (kPrintLog) {
        float ground_percentage = ground_indices.size() * 100.f / cloud_data->size();
        std::cout << std::fixed << std::setprecision(3) 
                  << "Ground points " << ground_indices.size() << "(" << ground_percentage 
                  << "%), ground score " << ground_evaluation << "." << std::endl 
                  << "Took [" << timePhase1 
                  << " / " << timeTotal << "s], [avg" 
                  << accumulated_run_time/num_received_msgs << "s]." 
                  << std::endl << std::endl;
    }

}

// implementation 2.
void FastGroundDetection::Segment(pcl::PointCloud<PointXYZIRT>::Ptr& cloud_data, 
            std::vector<int>* labels_out, float& ground_evaluation, bool use_intensity) 
{
    ordinary_cloud_used = false;
    custom_cloud_used = true;
    num_received_msgs++;
    if (kPrintLog) {
        std::cout << "Segmenting cloud with " << cloud_data->size() << " points... -------------- "
                  << "[msg #" << num_received_msgs << "] " << std::endl;
    }

    clock_t time_start = clock();

    // step01 register every and each point into corresponding sector!
    for (auto& sector : sectors_) {sector.Reset();}
    pcl::transformPointCloud(*cloud_data, *custom_cloud_in_base_, sensor_pose_in_base_);
    for (std::size_t i=0; i<cloud_data->size(); ++i)
    {
        // if (use_intensity) {cloud_data->points[i].intensity += kIntensityIncre;}
        if (use_intensity) {cloud_data->points[i].intensity = 10;}
        if (custom_cloud_in_base_->points[i].z > 1.0) continue;
        float ang_azimuth = std::atan2(cloud_data->points[i].y, cloud_data->points[i].x);
        while (ang_azimuth < 0) { ang_azimuth += 2*M_PI; } /* from [-pi,+pi] to [0, 2pi] */
        // float xy_range = std::sqrt( cloud_data->points[i].x*cloud_data->points[i].x
        //                             +cloud_data->points[i].y*cloud_data->points[i].y );

        int rowIdx=0, colIdx=0;
        rowIdx = cloud_data->points[i].ring;
        colIdx = floor(ang_azimuth*params_.kLidarHorizResInv);
        if (rowIdx < 0 || rowIdx >= params_.kLidarRows 
            || colIdx < 0 || colIdx >= params_.kLidarCols) {
            continue;
        }
        int sectorIdx = floor(colIdx/params_.kColsPerSector/*5*/);
        if (sectorIdx<0 || sectorIdx>=params_.kNumSectors) {
            continue;
        }

        // Note that here we must pass in point in base (or ground) frame!
        // Remember that the `intensity` represents x-y range!!!!
        pcl::PointXYZI point3D;
        point3D.x = custom_cloud_in_base_->points[i].x;
        point3D.y = custom_cloud_in_base_->points[i].y;
        point3D.z = custom_cloud_in_base_->points[i].z;
        point3D.intensity = std::sqrt(point3D.x*point3D.x
                                     +point3D.y*point3D.y);
        sectors_[sectorIdx].AddPoint(point3D, i/*original index*/, rowIdx/*bin id*/);
    }
    clock_t time_stop01 = clock();

    // step02 estimate ground line within each sector.
    extracted_lines_.clear();
    std::vector<int> ground_indices;
    for (std::size_t i=0; i<sectors_.size(); ++i) {
        sectors_[i].RunLineExtraction();
        auto& result = sectors_[i].GetGroundPointIndices(); 
        ground_indices.insert(ground_indices.end(), result.begin(), result.end());
        auto& geometry_lines = sectors_[i].GetExtractedLines();
        extracted_lines_.insert(extracted_lines_.end(), geometry_lines.begin(), geometry_lines.end());
        ground_distribution_[i].Reset();
        if (!result.empty()) {
            ground_distribution_[i].is_ground = 1;
        }
    }

    // step03 evaluate ground distribution score.
    ground_evaluation = EvaluateGroundScore();

    // step04 project labels to original `cloud_data`.
    labels_out->clear();
    labels_out->resize(cloud_data->size());
    std::fill(labels_out->begin(), labels_out->end(), 0); /* 0(NonGround), 1(Ground), 2(Others) */
    for (const auto& index : ground_indices) {
        (*labels_out)[index] = 1;
        if (use_intensity) {cloud_data->points[index].intensity=100;}
    }
    if (use_intensity) { /* expand intensity interval to [0, 255]. */
        cloud_data->points[0].intensity=0;
        cloud_data->points[1].intensity=255;
    } 

    clock_t time_end = clock();
    float timePhase1 = (float(time_stop01-time_start))/CLOCKS_PER_SEC;
    float timeTotal = (float(time_end-time_start))/CLOCKS_PER_SEC;
    accumulated_run_time += timeTotal;
    if (kPrintLog) {
        float ground_percentage = ground_indices.size() * 100.f / cloud_data->size();
        std::cout << std::fixed << std::setprecision(3) 
                  << "Ground points " << ground_indices.size() << "(" << ground_percentage 
                  << "%), ground score " << ground_evaluation << "." << std::endl 
                  << "Took [" << timePhase1 
                  << " / " << timeTotal << "s], [avg" 
                  << accumulated_run_time/num_received_msgs << "s]." 
                  << std::endl << std::endl;
    }

}

// implementation 3.
void FastGroundDetection::Segment(::cartographer::sensor::PointCloudWithIntensities& cloud_data, 
            std::vector<int>* labels_out, float& ground_evaluation,  bool use_intensity)
{
    ordinary_cloud_used = true;
    custom_cloud_used = false;
    num_received_msgs++;
    if (kPrintLog) {
        std::cout << std::endl
                  << "Segmenting cloud with " 
                  << cloud_data.points.size() << " points... -------------- "
                  << "[msg #" << num_received_msgs << "] " << std::endl;
    }

    clock_t time_start = clock();

    // step01 register every and each point into corresponding sector!
    for (auto& sector : sectors_) {sector.Reset();}
    common_cloud_in_base_->points.resize(cloud_data.points.size()); // transform point cloud.
    for (size_t i = 0; i < cloud_data.points.size(); ++i) 
    {
        Eigen::Matrix<float, 3, 1> pt (cloud_data.points[i].position[0], 
                                        cloud_data.points[i].position[1], 
                                        cloud_data.points[i].position[2]);
        common_cloud_in_base_->points[i].x = static_cast<float> (
                                sensor_pose_in_base_ (0, 0) * pt.coeffRef (0) 
                                + sensor_pose_in_base_ (0, 1) * pt.coeffRef (1) 
                                + sensor_pose_in_base_ (0, 2) * pt.coeffRef (2) 
                                + sensor_pose_in_base_ (0, 3));
        common_cloud_in_base_->points[i].y = static_cast<float> (
                                sensor_pose_in_base_ (1, 0) * pt.coeffRef (0) 
                                + sensor_pose_in_base_ (1, 1) * pt.coeffRef (1) 
                                + sensor_pose_in_base_ (1, 2) * pt.coeffRef (2) 
                                + sensor_pose_in_base_ (1, 3));
        common_cloud_in_base_->points[i].z = static_cast<float> (
                                sensor_pose_in_base_ (2, 0) * pt.coeffRef (0) 
                                + sensor_pose_in_base_ (2, 1) * pt.coeffRef (1) 
                                + sensor_pose_in_base_ (2, 2) * pt.coeffRef (2) 
                                + sensor_pose_in_base_ (2, 3));
        common_cloud_in_base_->points[i].intensity = cloud_data.intensities[i];
    }

    const auto& points_cloud = cloud_data.points;
    for (std::size_t i=0; i<cloud_data.points.size(); ++i)
    {
        if (use_intensity) {
            cloud_data.intensities[i] = 120; // 160-shallowBlue, 140-greenBlue, 120-green
            // /* control intensity range between [0,255]. */
            // /* z controls [20~255], more specifically, we cast [-2~10]m to [20~255] */
            // cloud_data.intensities[i] = kIntensityIncre
            //                             + (cloud_data.points[i].position.z() + 2) * 19.58;
            // if (cloud_data.intensities[i] > 255.0) {
            //     cloud_data.intensities[i] = 255.0;
            // }
            // if (cloud_data.intensities[i] < kIntensityIncre) {
            //     cloud_data.intensities[i] = kIntensityIncre;
            // }
        }
        if (common_cloud_in_base_->points[i].z > 1.0) continue;
        float ang_azimuth = std::atan2(points_cloud[i].position[1], points_cloud[i].position[0]);
        while (ang_azimuth < 0) { ang_azimuth += 2*M_PI; } /* from [-pi,+pi] to [0, 2pi] */
        float xy_range = std::sqrt( points_cloud[i].position[0]*points_cloud[i].position[0]
                                    +points_cloud[i].position[1]*points_cloud[i].position[1] );
        float ang_elevation = std::atan2(points_cloud[i].position[2], xy_range); /* naturally [-pi,+pi] */
        if (ang_elevation<params_.kLidarVertAngMin || ang_elevation>params_.kLidarVertAngMax) {
            continue;
        }

        int rowIdx=0, colIdx=0;
        rowIdx = round((ang_elevation - params_.kLidarVertFovMin) * params_.kLidarVertResInv);
        colIdx = floor(ang_azimuth*params_.kLidarHorizResInv);
        if (rowIdx < 0 || rowIdx >= params_.kLidarRows 
            || colIdx < 0 || colIdx >= params_.kLidarCols) {
            continue;
        }
        int sectorIdx = floor(colIdx/params_.kColsPerSector/*5*/);
        if (sectorIdx<0 || sectorIdx>=params_.kNumSectors) {
            continue;
        }

        // Note that here we must pass in point in base (or ground) frame!
        // Remember that the `intensity` represents x-y range!!!!
        pcl::PointXYZI point3D = common_cloud_in_base_->points[i];
        point3D.intensity = std::sqrt(point3D.x*point3D.x
                                     +point3D.y*point3D.y);
        sectors_[sectorIdx].AddPoint(point3D, i/*original index*/, rowIdx/*bin id*/);
    }
    clock_t time_stop01 = clock();

    // step02 extract line within each sector.
    extracted_lines_.clear();
    std::vector<int> ground_indices;
    for (std::size_t i=0; i<sectors_.size(); ++i) {
        sectors_[i].RunLineExtraction();
        auto& result = sectors_[i].GetGroundPointIndices(); 
        ground_indices.insert(ground_indices.end(), result.begin(), result.end());
        auto& geometry_lines = sectors_[i].GetExtractedLines();
        extracted_lines_.insert(extracted_lines_.end(), geometry_lines.begin(), geometry_lines.end());
        ground_distribution_[i].Reset();
        if (!result.empty()) {
            ground_distribution_[i].is_ground = 1;
        }
    }

    // step03 evaluate ground distribution score.
    ground_evaluation = EvaluateGroundScore();

    // step04 project labels to original `cloud_data`.
    labels_out->clear();
    labels_out->resize(cloud_data.points.size());
    std::fill(labels_out->begin(), labels_out->end(), 0); /* 0(NonGround), 1(Ground), 2(Others) */
    for (const auto& index : ground_indices) {
        (*labels_out)[index] = 1;
        if (use_intensity) {cloud_data.intensities[index]=kGroundIntensity;}
    }
    if (use_intensity) { /* expand intensity interval to [0, 255]. */
        cloud_data.intensities.back()=255;
    } 

    clock_t time_end = clock();
    float timePhase1 = (float(time_stop01-time_start))/CLOCKS_PER_SEC;
    float timeTotal = (float(time_end-time_start))/CLOCKS_PER_SEC;
    accumulated_run_time += timeTotal;
    if (kPrintLog) {
        float ground_percentage = ground_indices.size() * 100.f / cloud_data.points.size();
        std::cout << std::fixed << std::setprecision(3) 
                  << "Ground points " << ground_indices.size() << "(" << ground_percentage 
                  << "%), ground score " << ground_evaluation << "." << std::endl 
                  << "Took [" << timePhase1 
                  << " / " << timeTotal << "s], [avg" 
                  << accumulated_run_time/num_received_msgs << "s]." 
                  << std::endl;
    }

}

pcl::PointCloud<pcl::PointXYZI>::Ptr
FastGroundDetection::GetTransformedCommonCloud()
{
    if (custom_cloud_used) {
        std::cout << "ERROR! wrong cloud type used." << std::endl;
    }
    return common_cloud_in_base_;
}

pcl::PointCloud<PointXYZIRT>::Ptr
FastGroundDetection::GetTransformedCustomCloud()
{
    if (ordinary_cloud_used) {
        std::cout << "ERROR! wrong cloud type used." << std::endl;
    }
    return custom_cloud_in_base_;
}

std::vector<BasicLine>& FastGroundDetection::GetExtractedLines()
{
    return extracted_lines_;
}

float FastGroundDetection::EvaluateGroundScore()
{
    if (360 % params_.kGroundSectorScoreFov != 0
        || params_.kNumSectors % (360 / params_.kGroundSectorScoreFov) != 0 ) {
        std::cout << "kGroundSectorScoreFov must be able to divide kNumSectors, ERROR!" << std::endl;
        return 0;
    }

    int num_fovs = 360 / params_.kGroundSectorScoreFov;
    int num_sectors_of_unit_fov = params_.kNumSectors / num_fovs;
    int num_valid_ground_sectors = 0;
    for (int i=0; i < params_.kNumSectors; ++i) {
        // for one ground sector, evaluate it according to score-fov method.
        if (ground_distribution_[i].is_ground && !ground_distribution_[i].counted) {
            int start_pointer = i + params_.kNumSectors/2 - num_sectors_of_unit_fov/2;
            start_pointer = start_pointer % params_.kNumSectors;

            int j = 0;
            while (j<num_sectors_of_unit_fov) {
                int pointer = start_pointer+j;
                if (pointer >= params_.kNumSectors) pointer = pointer % params_.kNumSectors;
                if (ground_distribution_[pointer].is_ground) {
                    num_valid_ground_sectors++;
                    ground_distribution_[i].counted = true;
                    if (!ground_distribution_[pointer].counted) {
                        num_valid_ground_sectors++;
                        ground_distribution_[pointer].counted = true;
                    }
                    break;
                }
                j++;
            }
        }
    }

    return num_valid_ground_sectors * 1.0f / params_.kNumSectors;
}

}  // namespace cartographer_ros

