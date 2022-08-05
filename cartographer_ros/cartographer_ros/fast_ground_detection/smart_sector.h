/*
 * Copyright 2021 Guanhua WANG
 */

#ifndef EFFICIENT_ONLINE_SEGMENTATION_SECTOR_H_
#define EFFICIENT_ONLINE_SEGMENTATION_SECTOR_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <set>

// ******************************************************* //

namespace cartographer_ros {

struct IndexedPoint {
    pcl::PointXYZI point;
    int bin_index = -1;
    int original_index = -1;

    IndexedPoint();
    IndexedPoint(const pcl::PointXYZI& pointIn, const int& binIdxIn, const int& origIdxIn);

    bool operator==(const IndexedPoint& other) const;
    bool operator!=(const IndexedPoint& other) const;
    bool operator<(const IndexedPoint& other) const;
};

enum LineLabel { NAL /*not a label*/, GROUND, WALL };

struct BasicLine {
    LineLabel label;
    pcl::PointXYZI start_point;
    pcl::PointXYZI end_point;

    BasicLine();
    BasicLine(const LineLabel& labelIn,
                const pcl::PointXYZI startPoint, 
                const pcl::PointXYZI endPoint );
};

struct SmartLine {
    int start_bin = -1;
    int end_bin = -1;
    std::vector<int> bins_;
    std::vector<float> slopes_;

    // used for calculating slope in `local` frame.
    float locOriginX = 0;
    float locOriginY = 0;
    float locCosTheta = 1;
    float locSinTheta = 0;
    float locTanTheta = 0; // first two points slope (or tan).
    float last_point_x = 0;
    float last_point_y = 0;

    // only for calculating line parameters.
    bool isCalculated = false;
    pcl::PointXYZI start_point;
    pcl::PointXYZI end_point;
    float length_ = 0;
    float k_ = 0;
    float b_ = 0;

    // params.
    float kMaxGroundSlope;        // 10 degrees(0.1763)
    float kSameLineTolerance;     // 2 degree(0.035, around 0.1m/3m) 
    float kStartPointZTolerance;  // 0.5 meters.
    float kRingDistTolerance;     // 6 meters, adjacent two ground rings 
                                  // should not be too distant.

    SmartLine(const float& max_ground_slope=0.1763, 
              const float& ground_same_line_tolerance=0.035, 
              const float& start_point_z_tolerance=0.5, 
              const float& max_ring_dist=6);
    void Reset();
    bool TryAddNewBin(const int& bin_id, const pcl::PointXYZI& bin_point);
    BasicLine GetLine(const std::vector<pcl::PointXYZI>& bin_vec, 
                        const LineLabel& line_label);

};

// ******************************************************* //

/**
 * @brief The SmartSector class is an advanced implementation for line-extraction,
 * a line can be a ground line, a wall line, or a line attached on whatever object.
 * Note that inside the SmartSector class, PointXYZI::intensity is used for xy-range!
 */
class SmartSector {
  public:

    SmartSector();
    SmartSector(const int& this_sector_id,
            const unsigned int& n_bins,
            const float& sensor_height,
            const float& ground_same_line_tolerance = 0.035,
            const float& ground_slope_tolerance = 0.1763,
            const float& ground_intercept_tolerance = 0.5,
            const float& ground_pointline_dist = 0.1);

    // Pipeline.
    void Reset();
    void AddPoint(const pcl::PointXYZI& pointIn, const int& pointIdx, const int& binIdx);
    void RunLineExtraction();

    // Get results.
    std::vector<int>& GetGroundPointIndices();
    std::vector<BasicLine>& GetExtractedLines();

    // Query from outside. (0-NotGround, 1-Ground, 2-TODO)
    int IdentifyExternalPoint(const pcl::PointXYZI& pointIn, const int& binIdx);

  public:

   LineLabel JudgeLineLAbel(SmartLine& smartLine, const std::vector<pcl::PointXYZI>& binVec);

    // General params.
    int kSectorId = -1;
    int kNumBins;           // = 16 or 32;
    float kSensorHeight;    // = 0.0;

    // Identify ground.
    float kGroundSameLineTolerance;     // = 0.035;   // 2 degree(0.035, around 0.1m/3m) 
    float kGroundSlopeTolerance;        // = 0.1763;  // 10 degrees(0.1763), 2 degree(0.035), 1 degree(0.0175)
    float kGroundYInterceptTolerance;   // = 0.5;
    float kGroundPointLineDistThres;    // = 0.1;

    // frozen params.
    float kStrictGroundSlope = 0.03;     // 2 degree(0.035), 1 degree(0.0175)
    float kStrictYIntercept = 0.2;

    pcl::PointXYZI nanPoint;

    // all the following variables/containers need to be reset before next round.
    std::set<IndexedPoint> src_points_set_;
    std::vector<pcl::PointXYZI> sector_; 
    std::vector<int> sectorCounts_; 

    std::vector<BasicLine> extracted_lines_;

    std::vector<int> ground_bins_;
    std::vector<int> ground_points_indices_;

};

}  // namespace cartographer_ros

#endif // EFFICIENT_ONLINE_SEGMENTATION_SECTOR_H_
