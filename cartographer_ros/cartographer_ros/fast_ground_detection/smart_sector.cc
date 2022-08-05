/*
 * Copyright 2021 Guanhua WANG
 */

#include "cartographer_ros/fast_ground_detection/smart_sector.h"
#include <cmath>
#include <climits>

// ******************************************************* //

namespace cartographer_ros {

IndexedPoint::IndexedPoint(){}

IndexedPoint::IndexedPoint(const pcl::PointXYZI& pointIn, const int& binIdxIn, const int& origIdxIn)
    :point(pointIn), bin_index(binIdxIn), original_index(origIdxIn) {}

bool IndexedPoint::operator==(const IndexedPoint& other) const{
    return std::forward_as_tuple(bin_index, original_index) ==
        std::forward_as_tuple(other.bin_index, other.original_index);
}

bool IndexedPoint::operator!=(const IndexedPoint& other) const{
    return std::forward_as_tuple(bin_index, original_index) !=
        std::forward_as_tuple(other.bin_index, other.original_index);
}

bool IndexedPoint::operator<(const IndexedPoint& other) const{
    return std::forward_as_tuple(bin_index, original_index) <
        std::forward_as_tuple(other.bin_index, other.original_index);
}

BasicLine::BasicLine() :label(LineLabel::NAL)
{
    start_point.x=0; start_point.y=0; start_point.z=0; start_point.intensity=0;
    end_point.x=0; end_point.y=0; end_point.z=0; end_point.intensity=0;
}

BasicLine::BasicLine(const LineLabel& labelIn,
                    const pcl::PointXYZI startPoint, 
                    const pcl::PointXYZI endPoint )
    :label(labelIn),start_point(startPoint),end_point(endPoint) {}

SmartLine::SmartLine(const float& max_ground_slope, 
                    const float& ground_same_line_tolerance, 
                    const float& start_point_z_tolerance, 
                    const float& max_ring_dist)
    :kMaxGroundSlope(max_ground_slope), 
    kSameLineTolerance(ground_same_line_tolerance),
    kStartPointZTolerance(start_point_z_tolerance),
    kRingDistTolerance(max_ring_dist) {}

void SmartLine::Reset() 
{
    start_bin = -1;
    end_bin = -1;
    bins_.clear();
    slopes_.clear();
    locOriginX = 0;
    locOriginY = 0;
    locCosTheta = 1;
    locSinTheta = 0;
    locTanTheta = 0;
    last_point_x = 0;
    last_point_y = 0;
    isCalculated = false;
    length_ = 0;
    k_ = 0;
    b_ = 0;
}

bool SmartLine::TryAddNewBin(const int& bin_id, const pcl::PointXYZI& bin_point)
{
    /** @brief Note that we use the first two points define a Local Coordinate System,
     * all slopes btwn two adjacent points are calculated under this coordinate!
     * By doing so, we avoid singularity when tangent angle is near 90 degrees.
     * 
     * the coordinate of a point in Local is calculated by:
     * x' = (x-xOffset)*cos(theta) + (y-yOffset)*sin(theta);
     * y' = (y-yOffset)*cos(theta) - (x-xOffset)*sin(theta);
    **/

    if (bins_.size()==0) {
        if (bin_point.z < -kStartPointZTolerance 
            || bin_point.z > kStartPointZTolerance)
        {return false;}
        start_bin = bin_id;
        end_bin = bin_id;
        bins_.push_back(bin_id);
        locOriginX = bin_point.intensity; /*x-y range*/
        locOriginY = bin_point.z;
        return true;
    }
    else if (bins_.size()==1) {
        float deltaX = bin_point.intensity - locOriginX;
        float deltaY = bin_point.z - locOriginY;
        float dist = std::sqrt(deltaX*deltaX + deltaY*deltaY);
        if (dist > kRingDistTolerance) return false;
        end_bin = bin_id;
        bins_.push_back(bin_id);
        slopes_.push_back(0);
        locCosTheta = deltaX/dist;
        locSinTheta = deltaY/dist;
        locTanTheta = deltaY/deltaX;
        if (std::abs(locTanTheta) > kMaxGroundSlope) return false;
        last_point_x = dist;
        last_point_y = 0;
        return true;
    }
    else /*accumulated_bins>=2*/ {
        // calculate point coordinate in local frame.
        float curr_point_x = (bin_point.intensity-locOriginX)*locCosTheta 
                            +(bin_point.z-locOriginY)*locSinTheta;
        float curr_point_y = (bin_point.z-locOriginY)*locCosTheta 
                            -(bin_point.intensity-locOriginX)*locSinTheta;
        float curr_k = (curr_point_y - last_point_y) / (curr_point_x - last_point_x);
        for (const auto& existed_slope : slopes_) {
            if (std::abs(existed_slope - curr_k) > kSameLineTolerance
                || curr_point_x - last_point_x > kRingDistTolerance ) return false;
        }
        end_bin = bin_id;
        bins_.push_back(bin_id);
        slopes_.push_back(curr_k);
        last_point_x = curr_point_x;
        last_point_y = curr_point_y;
        return true;
    }

}

BasicLine SmartLine::GetLine(const std::vector<pcl::PointXYZI>& bin_vec, const LineLabel& line_label)
{
    if (bins_.size()<2 || bin_vec.size()<2) return BasicLine();
    if (start_bin<0 || (end_bin<=start_bin)) return BasicLine();

    if (!isCalculated) {
        start_point = bin_vec[start_bin];
        end_point = bin_vec[end_bin];
        if (!pcl::isFinite(start_point) || !pcl::isFinite(end_point)) return BasicLine();
    }

    return BasicLine(line_label,start_point,end_point);
}

// ******************************************************* //

SmartSector::SmartSector()
    :kSectorId(-1), kNumBins(0), kSensorHeight(0), 
    kGroundSameLineTolerance(0), kGroundSlopeTolerance(0), 
    kGroundYInterceptTolerance(0), kGroundPointLineDistThres(0)
{
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;
    sector_.resize(kNumBins);
    sectorCounts_.resize(kNumBins);
    Reset();
}

SmartSector::SmartSector(const int& this_sector_id,
                const unsigned int& n_bins,
                const float& sensor_height,
                const float& ground_same_line_tolerance,
                const float& ground_slope_tolerance,
                const float& ground_intercept_tolerance,
                const float& ground_pointline_dist) 
                : kSectorId(this_sector_id),
                kNumBins(n_bins),
                kSensorHeight(sensor_height),
                kGroundSameLineTolerance(ground_same_line_tolerance),
                kGroundSlopeTolerance(ground_slope_tolerance),
                kGroundYInterceptTolerance(ground_intercept_tolerance),
                kGroundPointLineDistThres(ground_pointline_dist)
{
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;
    sector_.resize(kNumBins);
    sectorCounts_.resize(kNumBins);
    Reset();

    // check initialization.
    int num_NanPoint = 0;
    for (auto& binPoint : sector_) {
        if (!pcl::isFinite(binPoint)) num_NanPoint++;
    }
    if (num_NanPoint == kNumBins) {
        // std::cout << "SmartSector[id=" << kSectorId 
        //             << "] initialized successfully. \n";
    } 
    else {
        std::cout << "SmartSector[id=" << kSectorId 
                    << "] initialization FAILED. ERROR! \n";
    }

}

void SmartSector::Reset()
{
    src_points_set_.clear();
    std::fill(sector_.begin(), sector_.end(), nanPoint);
    std::fill(sectorCounts_.begin(), sectorCounts_.end(), 0);

    extracted_lines_.clear();

    ground_bins_.clear();
    ground_points_indices_.clear();
}

void SmartSector::AddPoint(const pcl::PointXYZI& pointIn, const int& pointIdx, const int& binIdx)
{
    if (binIdx<0 || binIdx>=kNumBins) {
        // TODO: warn.
        return;
    }

    src_points_set_.emplace(pointIn,binIdx,pointIdx);

    // 'lowest as indicator' version.
    if (!pcl::isFinite(sector_[binIdx])) {
        sector_[binIdx] = pointIn;
        return;
    }
    if (pointIn.z < sector_[binIdx].z) sector_[binIdx] = pointIn;

}

void SmartSector::RunLineExtraction()
{
    // debug
    int num_nan_bins = 0;

    // extract line.
    SmartLine smart_line(kGroundSlopeTolerance,kGroundSameLineTolerance);
    for (std::size_t i=0; i<sector_.size(); ++i) {
        if ( !pcl::isFinite(sector_[i]) ) { num_nan_bins++; continue; }

        if (smart_line.TryAddNewBin(i,sector_[i])) { /*need to do nothing*/ }
        else { /* current line ended. */
            LineLabel line_label = JudgeLineLAbel(smart_line, sector_);
            if (line_label==LineLabel::GROUND) {
                extracted_lines_.push_back(smart_line.GetLine(sector_, line_label));
                ground_bins_.insert(ground_bins_.end(), smart_line.bins_.begin(), smart_line.bins_.end());
            }
            smart_line.Reset();
            /* start new line after reset. */
            smart_line.TryAddNewBin(i,sector_[i]);
        }

        if (i==sector_.size()-1 && smart_line.bins_.size()>=2) {
            LineLabel line_label = JudgeLineLAbel(smart_line, sector_);
            if (line_label==LineLabel::GROUND) {
                extracted_lines_.push_back(smart_line.GetLine(sector_, line_label));
                ground_bins_.insert(ground_bins_.end(), smart_line.bins_.begin(), smart_line.bins_.end());
            }
            smart_line.Reset();
        }
    }

    // label GROUND point, collect their indices.
    std::size_t binPointer = 0;
    int non_ground_bin = -1;
    ground_points_indices_.clear();
    if (!ground_bins_.empty()) {
        /* for src_points_set_, first bin order, then original index order. */
        for (const auto& point : src_points_set_) { 
            if (point.bin_index==non_ground_bin) continue;
            /* try to find corresponding bin for src point. */
            if (point.bin_index!=ground_bins_[binPointer])
            {
                while (point.bin_index!=ground_bins_[binPointer]) {
                    binPointer++;
                    if (binPointer>=ground_bins_.size()) {
                        binPointer = 0;
                        non_ground_bin = point.bin_index;
                        break;
                    }
                }
            }
            if (point.bin_index!=ground_bins_[binPointer]) continue;
            /* now, judge whether a point belongs to ground (according to height diff). */
            if (std::abs(point.point.z-sector_[point.bin_index].z)<kGroundPointLineDistThres) {
                ground_points_indices_.push_back(point.original_index);
            }
        }
    }

    // std::cout << "SmartSector[id=" << kSectorId 
    //         << "]: ground bins [" << ground_bins_.size() << "/" << kNumBins << "], "
    //         << "ground points [" << ground_points_indices_.size() << "]" << std::endl;

}

std::vector<int>& SmartSector::GetGroundPointIndices()
{
    return ground_points_indices_;
}

std::vector<BasicLine>& SmartSector::GetExtractedLines()
{
    return extracted_lines_;
}

int SmartSector::IdentifyExternalPoint(const pcl::PointXYZI& pointIn, const int& binIdx)
{
    if (binIdx<0 || binIdx>=kNumBins) {
        // TODO: warn.
        return 0;
    }

    for (const auto& bin : ground_bins_) {
        if (binIdx==bin) {
            if (std::abs(pointIn.z-sector_[bin].z)<kGroundPointLineDistThres) {
                return 1;
            }
        }
    }

    return 0;
}

LineLabel SmartSector::JudgeLineLAbel(SmartLine& smartLine, const std::vector<pcl::PointXYZI>& binVec)
{
    if (smartLine.bins_.size()<2 || binVec.size()<2) return LineLabel::NAL;
    if (smartLine.start_bin<0 || (smartLine.end_bin<=smartLine.start_bin)) return LineLabel::NAL;

    if (!smartLine.isCalculated) {
        smartLine.start_point = binVec[smartLine.start_bin];
        smartLine.end_point = binVec[smartLine.end_bin];
        if (!pcl::isFinite(smartLine.start_point) || !pcl::isFinite(smartLine.end_point)) return LineLabel::NAL;
        float deltaX = smartLine.end_point.intensity-smartLine.start_point.intensity;
        float deltaY = smartLine.end_point.z-smartLine.start_point.z;
        // smartLine.k_ = deltaY/deltaX;           // strategy A. 
        smartLine.k_ = smartLine.locTanTheta;   // strategy B. 
        smartLine.b_ = smartLine.start_point.z - smartLine.k_ * smartLine.start_point.intensity;
        smartLine.length_ = std::sqrt(deltaY*deltaY+deltaX*deltaX);
        smartLine.isCalculated = true;
    }

    // check whether this line is a ground line.
    if ((smartLine.bins_.size()>2
         &&std::abs(smartLine.k_)<kGroundSlopeTolerance
         && std::abs(smartLine.b_)<kGroundYInterceptTolerance)
        || 
        (smartLine.bins_.size()==2
         &&std::abs(smartLine.k_)<kStrictGroundSlope
         && std::abs(smartLine.start_point.z)<kStrictYIntercept
         && std::abs(smartLine.end_point.z)<kStrictYIntercept
         && std::abs(smartLine.b_)<0.5*kGroundYInterceptTolerance) )
    {
        return LineLabel::GROUND;
    }

    return LineLabel::NAL;
}

}  // namespace cartographer_ros





