/**
 * @file RobotMotionMapUpdater.cpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 由位姿协方差生成各栅格方差增量矩阵，并调用 ElevationMap::update。
 */
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/ParameterHelpers.hpp"

#include <kindr/Core>

namespace elevation_mapping {

RobotMotionMapUpdater::RobotMotionMapUpdater(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)), previousUpdateTime_(node_->now()), covarianceScale_(1.0) {
  previousReducedCovariance_.setZero();
}

RobotMotionMapUpdater::~RobotMotionMapUpdater() = default;

bool RobotMotionMapUpdater::readParameters() {
  covarianceScale_ = declareOrGetParameter(node_.get(), "robot_motion_map_update.covariance_scale", 1.0);
  return true;
}

bool RobotMotionMapUpdater::update(ElevationMap& map, const Pose& robotPose, const PoseCovariance& robotPoseCovariance,
                                   const rclcpp::Time& time) {
  const PoseCovariance robotPoseCovarianceScaled = covarianceScale_ * robotPoseCovariance;

  if (previousUpdateTime_ == time) {
    return true;
  }

  grid_map::Size size = map.getRawGridMap().getSize();
  grid_map::Matrix varianceUpdate(size(0), size(1));
  grid_map::Matrix horizontalVarianceUpdateX(size(0), size(1));
  grid_map::Matrix horizontalVarianceUpdateY(size(0), size(1));
  grid_map::Matrix horizontalVarianceUpdateXY(size(0), size(1));

  ReducedCovariance reducedCovariance;
  computeReducedCovariance(robotPose, robotPoseCovarianceScaled, reducedCovariance);
  ReducedCovariance relativeCovariance;
  computeRelativeCovariance(robotPose, reducedCovariance, relativeCovariance);

  Covariance positionCovariance = relativeCovariance.topLeftCorner<3, 3>();
  Covariance rotationCovariance(Covariance::Zero());
  rotationCovariance(2, 2) = relativeCovariance(3, 3);

  kindr::RotationMatrixPD mapToRobotRotation = kindr::RotationMatrixPD(robotPose.getRotation().inverted() * map.getPose().getRotation());
  kindr::RotationMatrixPD mapToPreviousRobotRotationInverted =
      kindr::RotationMatrixPD(previousRobotPose_.getRotation().inverted() * map.getPose().getRotation()).inverted();

  Eigen::Matrix3d translationJacobian = -mapToRobotRotation.matrix().transpose();

  Eigen::Vector3f translationVarianceUpdate =
      (translationJacobian * positionCovariance * translationJacobian.transpose()).diagonal().cast<float>();

  const kindr::Position3D positionRobotToMap =
      map.getPose().getRotation().inverseRotate(map.getPose().getPosition() - previousRobotPose_.getPosition());

  auto& heightLayer = map.getRawGridMap()["elevation"];

  for (unsigned int i = 0; i < static_cast<unsigned int>(size(0)); ++i) {
    for (unsigned int j = 0; j < static_cast<unsigned int>(size(1)); ++j) {
      kindr::Position3D cellPosition;
      const auto height = heightLayer(i, j);
      if (std::isfinite(height)) {
        grid_map::Position position;
        map.getRawGridMap().getPosition({i, j}, position);
        cellPosition = {position.x(), position.y(), height};

        const Eigen::Matrix3d rotationJacobian =
            -kindr::getSkewMatrixFromVector((positionRobotToMap + cellPosition).vector()) * mapToPreviousRobotRotationInverted.matrix();

        const Eigen::Matrix2f rotationVarianceUpdate =
            (rotationJacobian * rotationCovariance * rotationJacobian.transpose()).topLeftCorner<2, 2>().cast<float>();

        varianceUpdate(i, j) = translationVarianceUpdate.z();
        horizontalVarianceUpdateX(i, j) = translationVarianceUpdate.x() + rotationVarianceUpdate(0, 0);
        horizontalVarianceUpdateY(i, j) = translationVarianceUpdate.y() + rotationVarianceUpdate(1, 1);
        horizontalVarianceUpdateXY(i, j) = rotationVarianceUpdate(0, 1);
      } else {
        varianceUpdate(i, j) = std::numeric_limits<float>::infinity();
        horizontalVarianceUpdateX(i, j) = std::numeric_limits<float>::infinity();
        horizontalVarianceUpdateY(i, j) = std::numeric_limits<float>::infinity();
        horizontalVarianceUpdateXY(i, j) = std::numeric_limits<float>::infinity();
      }
    }
  }

  map.update(varianceUpdate, horizontalVarianceUpdateX, horizontalVarianceUpdateY, horizontalVarianceUpdateXY, time);
  previousReducedCovariance_ = reducedCovariance;
  previousRobotPose_ = robotPose;
  return true;
}

bool RobotMotionMapUpdater::computeReducedCovariance(const Pose& robotPose, const PoseCovariance& robotPoseCovariance,
                                                     ReducedCovariance& reducedCovariance) {
  kindr::EulerAnglesZyxPD eulerAngles(robotPose.getRotation());
  double tanOfPitch = tan(eulerAngles.pitch());
  Eigen::Matrix<double, 1, 3> yawJacobian(cos(eulerAngles.yaw()) * tanOfPitch, sin(eulerAngles.yaw()) * tanOfPitch, 1.0);
  Eigen::Matrix<double, 4, 6> jacobian;
  jacobian.setZero();
  jacobian.topLeftCorner(3, 3).setIdentity();
  jacobian.bottomRightCorner(1, 3) = yawJacobian;

  reducedCovariance = jacobian * robotPoseCovariance * jacobian.transpose();
  return true;
}

bool RobotMotionMapUpdater::computeRelativeCovariance(const Pose& robotPose, const ReducedCovariance& reducedCovariance,
                                                      ReducedCovariance& relativeCovariance) {
  const kindr::RotationVectorPD rotationVector_I_B(robotPose.getRotation());
  const kindr::RotationVectorPD rotationVector_I_tilde_B(0.0, 0.0, rotationVector_I_B.vector().z());
  const kindr::RotationMatrixPD R_I_tilde_B(rotationVector_I_tilde_B);

  kindr::Position3D positionInRobotFrame =
      previousRobotPose_.getRotation().inverseRotate(robotPose.getPosition() - previousRobotPose_.getPosition());
  kindr::Velocity3D v_Delta_t(positionInRobotFrame);

  Jacobian F;
  F.setIdentity();
  F.topRightCorner(3, 1) = kindr::getSkewMatrixFromVector(Eigen::Vector3d(0.0, 0.0, 1.0)) * R_I_tilde_B.matrix() * v_Delta_t.vector();

  Jacobian inv_G_Delta_t;
  inv_G_Delta_t.setZero();
  inv_G_Delta_t(3, 3) = 1.0;
  Jacobian inv_G_transpose_Delta_t(inv_G_Delta_t);
  inv_G_Delta_t.topLeftCorner(3, 3) = R_I_tilde_B.matrix().transpose();
  inv_G_transpose_Delta_t.topLeftCorner(3, 3) = R_I_tilde_B.matrix();

  relativeCovariance = inv_G_Delta_t * (reducedCovariance - F * previousReducedCovariance_ * F.transpose()) * inv_G_transpose_Delta_t;

  return true;
}

}  // namespace elevation_mapping
