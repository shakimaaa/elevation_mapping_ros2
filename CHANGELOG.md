# Changelog

> Category hints: `Added`, `Changed`, `Deprecated`, `Removed`, `Fixed`  
> (Current project custom sections also include: `Bug`)


All notable changes to this project will be documented in this file.

## [v0.0.3] - 2026.5.8
### Added
- Added a built-in elevation postprocessing hierarchy (inpaint -> repeated median denoise -> `hs1` light Gaussian -> `hs2` virtual-floor branch) with ROS parameters under `postprocessor_*`.
- Added two postprocessed output layers by default: `elevation_hs1` and `elevation_hs2`.
- Added sign-separated obstacle masks for the virtual-floor branch with configurable threshold `postprocessor_delta_height_mask_threshold` (`delta > +tau` for stepping stones, `delta < -tau` for gaps).
- Added postprocessed map caching in `ElevationMap` and callback wiring in `PostprocessorPool` so downstream modules can consume postprocessed layers directly.
- Added sampler parameter `elevation_sampling.use_postprocessed_map` to allow `ElevationMapRobotFrameSampler` to read from the cached postprocessed map.

### Fixed
- Fixed OpenCV runtime failures on systems where `cv::medianBlur` does not support `CV_32F` input by introducing a float32-compatible median filter path.
- Fixed stepping-stone/gap handling semantics by applying dilation only to positive obstacle masks, preventing gap regions from being unintentionally max-filled.
- Fixed sampled cloud layer access for `elevation_hs2` by enabling direct sampling from postprocessed maps (instead of raw/fused-only sources).

### Changed
- Changed built-in postprocessing fallback behavior: if external filter-chain configuration is unavailable, processing now falls back to the built-in hierarchy instead of forwarding raw map only.
- Changed robot configuration defaults for postprocessed sampling workflows by supporting `use_postprocessed_map: true` with `layer_name: elevation_hs2`.

---

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