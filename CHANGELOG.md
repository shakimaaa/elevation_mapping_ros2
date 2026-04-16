# Changelog

> Category hints: `Added`, `Changed`, `Deprecated`, `Removed`, `Fixed`  
> (Current project custom sections also include: `Bug`)


All notable changes to this project will be documented in this file.

## [v0.0.2] - 2026.4.16
### Added
- Added local rectangular elevation sampling parameters (`lateral_samples`, `longitudinal_samples`, `lateral_length`, `longitudinal_length`) for robot-centered sampled clouds.
- Added the `rotate_output_with_robot_attitude` option to control whether sampled cloud outputs fully follow robot attitude changes.
- Added the `invalid_height_fill_mode` option to control how missing elevation samples are filled.
- Added an optional sampled-point index `MarkerArray` publisher for RViz visualization.
- Added the Python debugging script `print_elevation_sampled_cloud.py` to print ordered sampled cloud points, show the grid layout, and optionally save CSV output.
### Fixed
- Reduced TF timeout spam by enabling a dedicated TF listener thread (`TransformListener` spin thread + `tf_buffer_->setUsingDedicatedThread(true)`), which stabilizes `lookupTransform(..., timeout)` during point cloud processing.
- Fixed sampled-cloud body-frame spacing by defining the sampling lattice in the robot local frame and querying elevation from the map.
- Fixed point-cloud subscription callback scheduling by assigning dedicated callback groups to cloud subscriptions.
- Fixed missing-height fallback behavior by supporting both previous-sample fill and body-to-ground height fill with consistent sign handling.
### Changed
- Changed elevation sampled-cloud generation from stride-based whole-map traversal to robot-centered local rectangular sampling.
- Changed sampled-cloud publication to support optional body-frame point index markers and dedicated debugging tooling.
- Changed the sampler execution path to reduce contention and runtime overhead at higher publish rates.
- Changed sampled-cloud output behavior to support both fixed local-grid output and full attitude-following output.

---

## [v0.0.1] - 2026.4.16
### Added
- Refactored `elevation_mapping` for ROS 2.
- Added sequential point cloud sampling.

---

## [v0.0.0] - 2026.4.6
### Added
- Initial version.