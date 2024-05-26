#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include "helper.h"

const double ICP_MAX_CORRESPONDENCE_DISTANCE = 2.0;
const double ICP_TRANSFORMATION_EPSILON = 0.001;
const double ICP_EUCLIDEAN_FITNESS_EPSILON = 0.5;
const double ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD = 10.0;

const double NDT_TRANSFORMATION_EPSILON = 0.001;
const double NDT_RESOLUTION = 1.0;
const double NDT_STEP_SIZE = 0.5;

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations){
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	auto& rotation = startingPose.rotation;
	auto& position = startingPose.position;
	Eigen::Matrix4d starting_pose_transformation_matrix = transform3D(
		rotation.yaw, rotation.pitch, rotation.roll,
		position.x, position.y, position.z
	);

	PointCloudT::Ptr source_transformed{new PointCloudT};

	pcl::transformPointCloud(*source, *source_transformed,
													starting_pose_transformation_matrix);

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(source_transformed);
	icp.setInputTarget(target);
	icp.setMaxCorrespondenceDistance(2);

  if (ICP_TRANSFORMATION_EPSILON > 0) {
  	icp.setTransformationEpsilon(ICP_TRANSFORMATION_EPSILON);
  }

  if (ICP_EUCLIDEAN_FITNESS_EPSILON > 0) {
  	icp.setEuclideanFitnessEpsilon(ICP_EUCLIDEAN_FITNESS_EPSILON);
  }

  if (ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD > 0) {
  	icp.setRANSACOutlierRejectionThreshold(ICP_RANSAC_OUTLIER_REJECTION_THRESHOLD);
  }

	PointCloudT::Ptr cloud_icp(new PointCloudT);
	icp.align(*cloud_icp);

	if (icp.hasConverged()) {
		return icp.getFinalTransformation().cast<double>() * starting_pose_transformation_matrix;
	}

	return transformation_matrix;
}

Eigen::Matrix4d NDT(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt, PointCloudT::Ptr source, Pose startingPose, int iterations){
	auto& rotation = startingPose.rotation;
	auto& position = startingPose.position;
	Eigen::Matrix4f starting_pose_transformation_matrix = transform3D(
		rotation.yaw, rotation.pitch, rotation.roll,
		position.x, position.y, position.z
	).cast<float>();

	ndt.setMaximumIterations(iterations);
	ndt.setInputSource(source);

	PointCloudT::Ptr cloud_ndt(new PointCloudT);
	ndt.align(*cloud_ndt, starting_pose_transformation_matrix);

	return ndt.getFinalTransformation().cast<double>();
}

pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> initializeNDT(PointCloudT::Ptr target) {
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(NDT_TRANSFORMATION_EPSILON);
	ndt.setInputTarget(target);
	ndt.setResolution(NDT_RESOLUTION);
	ndt.setStepSize(NDT_STEP_SIZE);

	return ndt;
}
