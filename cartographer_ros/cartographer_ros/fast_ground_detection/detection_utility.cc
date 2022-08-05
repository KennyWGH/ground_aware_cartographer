/*
 * Copyright 2021 Guanhua WANG
 */

#include "cartographer_ros/fast_ground_detection/detection_utility.h"
#include <iostream>

namespace cartographer_ros {

DetectionParams::DetectionParams() {}

// DetectionParams& DetectionParams::operator=(const DetectionParams& other)
void DetectionParams::operator=(const DetectionParams& other)
{
    kLidarRows = other.kLidarRows;
    kLidarCols = other.kLidarCols;
    kLidarHorizRes = other.kLidarHorizRes;
    kLidarVertRes = other.kLidarVertRes;
    kLidarVertFovMax = other.kLidarVertFovMax;
    kLidarVertFovMin = other.kLidarVertFovMin;
    kLidarProjectionError = other.kLidarProjectionError;

    kLidarHorizResInv = other.kLidarHorizResInv;
    kLidarVertResInv = other.kLidarVertResInv;
    kLidarVertAngMin = other.kLidarVertAngMin;
    kLidarVertAngMax = other.kLidarVertAngMax;

    kNumSectors = other.kNumSectors;
    kColsPerSector = other.kColsPerSector;
    kSensorHeight = other.kSensorHeight;
    kSensorRoll = other.kSensorRoll;
    kSensorPitch = other.kSensorPitch;
    kExtrinsicTrans = other.kExtrinsicTrans;
    kExtrinsicRot = other.kExtrinsicRot;
    kExtrinsicTF = other.kExtrinsicTF;
    kBaseToSensor = other.kBaseToSensor;

    kGroundSameLineTolerance = other.kGroundSameLineTolerance;
    kGroundSlopeTolerance = other.kGroundSlopeTolerance;
    kGroundYInterceptTolerance = other.kGroundYInterceptTolerance;
    kGroundPointLineDistThres = other.kGroundPointLineDistThres;

    kGroundSectorScoreFov = other.kGroundSectorScoreFov;

    // PrintAllParams("Copied parameters");
}

bool DetectionParams::UpdateInternalParams()
{
    bool is_user_params_correct = true;

    kLidarHorizResInv = 1/kLidarHorizRes; 
    kLidarVertResInv = 1/kLidarVertRes; 
    kLidarVertAngMin = kLidarVertFovMin - kLidarProjectionError;
    kLidarVertAngMax = kLidarVertFovMax + kLidarProjectionError; 

    kColsPerSector = kLidarCols/kNumSectors;

    // Check: the rotation matrix must be a special orthogonal matrix!
    {
        std::cout << std::endl;
        std::cout << "###################### CHECK ROTATION MATRIX ###################### " << std::endl;
        float det = kExtrinsicRot.determinant();
        std::cout << "## CHECK: RotMatrix.determinant(): " << det << std::endl << std::endl;
        Eigen::Matrix3f multipMat = kExtrinsicRot * kExtrinsicRot.transpose();
        std::cout << "## CHECK: RotMatrix*RotMatrix.transpose(): " << std::endl 
                  << multipMat << std::endl;
        multipMat = multipMat - Eigen::Matrix3f::Identity();
        float multipValue = std::abs(multipMat(0,0)) + std::abs(multipMat(0,1)) + std::abs(multipMat(0,2))
                            + std::abs(multipMat(1,0)) + std::abs(multipMat(1,1)) + std::abs(multipMat(1,2))
                            + std::abs(multipMat(2,0)) + std::abs(multipMat(2,1)) + std::abs(multipMat(2,2));
        std::cout << "## CHECK: RotMatrix*RotMatrix.transpose() abs sum: " << multipValue << std::endl;
        if (multipValue>0.01 || std::abs(det-1)>0.01 ) {
            is_user_params_correct = false;
            std::cout << std::endl;
            std::cout << " ############################ ERROR! ############################ " << std::endl;
            std::cout << " the extrinsic rotation matrix must be a SO3 matrix, which is not!" << std::endl;
            std::cout << " ############################ ERROR! ############################" << std::endl;
            std::cout << std::endl;
        }
        else {
            std::cout << "## CHECK: rotation matrix is cool. " << std::endl;
        }
        std::cout << "###################### CHECK ROTATION MATRIX ###################### " << std::endl;
        std::cout << std::endl;
    }

    kExtrinsicTF = Eigen::Matrix4f::Zero();
    kExtrinsicTF.block<3,3>(0,0) = kExtrinsicRot;
    kExtrinsicTF.block<3,1>(0,3) = kExtrinsicTrans;
    kBaseToSensor = kExtrinsicTF;

    PrintAllParams("Updated parameters");

    if (is_user_params_correct) {return true;}
    else {return false;}
}

void DetectionParams::PrintAllParams(const std::string& title)
{
    std::cout << "########## " << title << ": " << std::endl
            << "kLidarRows: " << kLidarRows << std::endl
            << "kLidarCols: " << kLidarCols << std::endl
            << "kLidarHorizRes: " << kLidarHorizRes << std::endl
            << "kLidarVertRes: " << kLidarVertRes << std::endl
            << "kLidarVertFovMax: " << kLidarVertFovMax << std::endl
            << "kLidarVertFovMin: " << kLidarVertFovMin << std::endl
            << "kLidarProjectionError: " << kLidarProjectionError << std::endl
            << " " << std::endl
            << "kLidarHorizResInv: " << kLidarHorizResInv << std::endl
            << "kLidarVertResInv: " << kLidarVertResInv << std::endl
            << "kLidarVertAngMin: " << kLidarVertAngMin << std::endl
            << "kLidarVertAngMax: " << kLidarVertAngMax << std::endl
            << " " << std::endl
            << "kNumSectors: " << kNumSectors << std::endl
            << "kColsPerSector: " << kColsPerSector << std::endl
            << "kSensorHeight: " << kSensorHeight << std::endl
            << "kSensorRoll: " << kSensorRoll << std::endl
            << "kSensorPitch: " << kSensorPitch << std::endl
            << "kExtrinsicTrans: " << std::endl 
            << kExtrinsicTrans << std::endl 
            << "kExtrinsicRot: " << std::endl 
            << kExtrinsicRot << std::endl 
            << "kBaseToSensor: " << std::endl 
            << kBaseToSensor.matrix() << std::endl 
            << " " << std::endl
            << "kGroundSameLineTolerance: " << kGroundSameLineTolerance << std::endl 
            << "kGroundSlopeTolerance: " << kGroundSlopeTolerance << std::endl 
            << "kGroundYInterceptTolerance: " << kGroundYInterceptTolerance << std::endl 
            << "kGroundPointLineDistThres: " << kGroundPointLineDistThres << std::endl 
            << " " << std::endl
            << "kGroundSectorScoreFov: " << kGroundSectorScoreFov << std::endl 
            << " " << std::endl;
}

}  // namespace cartographer_ros
