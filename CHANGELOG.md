# Changelog

All notable changes to this project will be documented in this file.

## [v0.0.1] - 2026.4.16
### Added
- Refactor the elevationmap to ROS2 version
- Add point cloud sequential sampling function
### Fixed
- Fixed TF timeout spam by enabling a dedicated TF listener thread (`TransformListener` spin thread + `tf_buffer_->setUsingDedicatedThread(true)`), which stabilizes `lookupTransform(..., timeout)` during point cloud processing.