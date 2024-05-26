#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Core>
#include "helper.h"

enum LocalizationMethod{ Icp, Ndt };

extern const double ICP_MAX_CORRESPONDENCE_DISTANCE;
extern const double ICP_TRANSFORMATION_EPSILON;
extern const double ICP_EUCLIDEAN_FITNESS_EPSILON;
extern const double ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD;

extern const double NDT_TRANSFORMATION_EPSILON;
extern const double NDT_RESOLUTION;
extern const double NDT_STEP_SIZE;

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations);

Eigen::Matrix4d NDT(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt, PointCloudT::Ptr source, Pose startingPose, int iterations);

pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> initializeNDT(PointCloudT::Ptr target);

#endif // LOCALIZATION_H
