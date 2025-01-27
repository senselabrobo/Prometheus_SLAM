#ifndef VOXBLOX_INCLUDE_VOXBLOX_INTEGRATOR_PROJECTIVE_TSDF_INTEGRATOR_INL_H_
#define VOXBLOX_INCLUDE_VOXBLOX_INTEGRATOR_PROJECTIVE_TSDF_INTEGRATOR_INL_H_

#include <algorithm>
#include <iostream>
#include <list>
#include <vector>

#include "voxblox/core/block.h"

namespace voxblox {
template <InterpolationScheme interpolation_scheme>
ProjectiveTsdfIntegrator<interpolation_scheme>::ProjectiveTsdfIntegrator(
    const voxblox::TsdfIntegratorBase::Config &config,
    voxblox::Layer<voxblox::TsdfVoxel> *layer)
    : TsdfIntegratorBase(config, layer),
      horizontal_resolution_(config.sensor_horizontal_resolution),
      vertical_resolution_(config.sensor_vertical_resolution),
      vertical_fov_rad_(config.sensor_vertical_field_of_view_degrees * M_PI /
                        180.0),
      range_image_(config.sensor_vertical_resolution,
                   config.sensor_horizontal_resolution),
      color_image_(config.sensor_vertical_resolution,
                   config.sensor_horizontal_resolution),
      num_voxels_per_block_(layer_->voxels_per_side() *
                            layer_->voxels_per_side() *
                            layer_->voxels_per_side()),
      ray_intersections_per_distance_squared_(
          voxel_size_ * horizontal_resolution_ /
          (2 * M_PI)  // horizontal point density
          * voxel_size_ * vertical_resolution_ /
          vertical_fov_rad_)  // vertical point density
{
  CHECK_GT(horizontal_resolution_, 0)
      << "The horizontal sensor resolution must be a positive integer";
  CHECK_GT(vertical_resolution_, 0)
      << "The vertical sensor resolution must be a positive integer";
  CHECK_GT(vertical_fov_rad_, 0)
      << "The vertical field of view of the sensor must be a positive float";
  CHECK(config_.use_const_weight) << "Scaling the weight by the inverse square "
                                     "depth is (not yet) supported.";
}

template <InterpolationScheme interpolation_scheme>
void ProjectiveTsdfIntegrator<interpolation_scheme>::integratePointCloud(
    const Transformation &T_G_C, const Pointcloud &points_C,
    const Colors &colors, const bool freespace_points, const bool deintegrate) {
  // Freespace points are not yet supported
  CHECK(!freespace_points) << "Freespace points are not yet supported for the "
                              "Projective TSDF Integrator";

  // Construct the range image and select the blocks it affects
  voxblox::IndexSet touched_block_indices;
  timing::Timer range_image_and_block_selector_timer(
      "range_image_and_block_selector_timer");
  parsePointcloud(T_G_C, points_C, colors, &range_image_, &color_image_,
                  &touched_block_indices);
  range_image_and_block_selector_timer.Stop();

  // Process all blocks
  timing::Timer integration_timer("integration_timer");
  if (config_.integrator_threads == 1) {
    updateTsdfBlocks(T_G_C, range_image_,
                     colors.size() ? &color_image_ : nullptr,
                     touched_block_indices, deintegrate);
  } else {
    std::vector<voxblox::IndexSet> block_index_subsets(
        config_.integrator_threads);
    size_t idx = 0;
    for (const auto &block_index : touched_block_indices) {
      block_index_subsets[idx % config_.integrator_threads].emplace(
          block_index);
      idx++;
    }
    std::list<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(
          &ProjectiveTsdfIntegrator::updateTsdfBlocks, this, T_G_C,
          range_image_, colors.size() ? &color_image_ : nullptr,
          block_index_subsets[i], deintegrate);
    }
    for (std::thread &integration_thread : integration_threads) {
      integration_thread.join();
    }
  }
  integration_timer.Stop();
}

template <InterpolationScheme interpolation_scheme>
Pointcloud
ProjectiveTsdfIntegrator<interpolation_scheme>::getReprojectedPointcloud() {
  Pointcloud reprojected_cloud;
  reprojected_cloud.reserve(vertical_resolution_ * horizontal_resolution_);
  for (int h = 0; h < vertical_resolution_; ++h) {
    for (int w = 0; w < horizontal_resolution_; ++w) {
      reprojected_cloud.emplace_back(range_image_(h, w) * imageToBearing(h, w));
    }
  }
  return reprojected_cloud;
}

template <InterpolationScheme interpolation_scheme>
void ProjectiveTsdfIntegrator<interpolation_scheme>::parsePointcloud(
    const Transformation &T_G_C, const Pointcloud &points_C,
    const Colors &colors, Eigen::MatrixXf *range_image, ColorImage *color_image,
    voxblox::IndexSet *touched_block_indices) const {
  CHECK_NOTNULL(range_image);
  CHECK_NOTNULL(color_image);
  CHECK_NOTNULL(touched_block_indices);
  Eigen::MatrixXf color_cnt_image;
  if (colors.size()) {
    CHECK_EQ(points_C.size(), colors.size());
    color_cnt_image = Eigen::MatrixXf(color_image->rows(), color_image->cols());
    color_cnt_image.setZero();
    color_image->setConstant(Color());
  }
  if (config_.use_missing_points_for_clearing) {
    constexpr float kMaxMissingPointsToClearRatio = 0.3;
    const float total_resolution =
        horizontal_resolution_ * vertical_resolution_;
    const float min_num_points_for_clearing =
        (1.f - kMaxMissingPointsToClearRatio) * total_resolution;
    if (min_num_points_for_clearing < points_C.size()) {
      range_image->setConstant(config_.max_ray_length_m +
                               config_.default_truncation_distance);
      // Hack to account for AirSim regularly not returning points for the
      // center columns of the LiDAR
      range_image->block(0, horizontal_resolution_ / 2, vertical_resolution_, 2)
          .setZero();
    } else {
      range_image->setZero();
    }
  } else {
    range_image->setZero();
  }
  voxblox::Point t_G_C_scaled = T_G_C.getPosition() * layer_->block_size_inv();
  for (size_t point_idx = 0; point_idx < points_C.size(); point_idx++) {
    auto const &point_C = points_C[point_idx];

    // Compute the point's bearing vector
    float distance = point_C.norm();
    if (distance < kFloatEpsilon) {
      // Avoid divisions by zero
      continue;
    }
    const Point point_C_bearing = point_C / distance;

    // Get the bearing vector's coordinates in the range image
    // NOTE: In this first implementation we only update the range image at its
    //       nearest neighbor coordinate
    int h, w;
    if (!bearingToImage(point_C_bearing, &h, &w)) {
      continue;
    }

    // Set the range image distance
    if (range_image->operator()(h, w) < kFloatEpsilon) {
      range_image->operator()(h, w) = distance;
    } else {
      // If the range pixel has already been updated by another ray,
      // we resolve the conflict by taking the minimum distance
      range_image->operator()(h, w) =
          std::min(range_image->operator()(h, w), distance);
    }

    if (colors.size())
      color_image->operator()(h, w) = Color::blendTwoColors(
          colors[point_idx], 1 / (color_cnt_image.operator()(h, w) + 1),
          color_image->operator()(h, w),
          (color_cnt_image.operator()(h, w) /
           (color_cnt_image.operator()(h, w) + 1)));

    // Mark the blocks hit by this ray
    if (config_.min_ray_length_m <= distance &&
        distance <= config_.max_ray_length_m) {
      Point point_G_scaled =
          T_G_C *
          (point_C_bearing * (distance + config_.default_truncation_distance)) *
          layer_->block_size_inv();
      Point t_G_C_truncated_scaled;
      if (config_.voxel_carving_enabled) {
        t_G_C_truncated_scaled = t_G_C_scaled;
      } else {
        t_G_C_truncated_scaled =
            point_G_scaled - T_G_C * point_C_bearing *
                                 config_.default_truncation_distance *
                                 layer_->block_size_inv();
      }
      voxblox::GlobalIndex block_index;
      voxblox::RayCaster ray_caster(point_G_scaled, t_G_C_truncated_scaled);
      while (ray_caster.nextRayIndex(&block_index)) {
        touched_block_indices->insert(block_index.cast<IndexElement>());
      }
    }
  }

  // Allocate all the blocks
  // NOTE: The layer's BlockHashMap is not thread safe, so doing it here before
  //       we start the multi-threading makes life easier
  for (const BlockIndex &block_index : *touched_block_indices) {
    layer_->allocateBlockPtrByIndex(block_index);
  }
}

template <InterpolationScheme interpolation_scheme>
void ProjectiveTsdfIntegrator<interpolation_scheme>::updateTsdfBlocks(
    const Transformation &T_G_C, const Eigen::MatrixXf &range_image,
    ColorImage *color_image, const voxblox::IndexSet &touched_block_indices,
    const bool deintegrate) {
  for (const voxblox::BlockIndex &block_index : touched_block_indices) {
    voxblox::Block<TsdfVoxel>::Ptr block_ptr =
        layer_->getBlockPtrByIndex(block_index);
    block_ptr->updated().set();

    for (size_t linear_index = 0u; linear_index < num_voxels_per_block_;
         ++linear_index) {
      TsdfVoxel *tsdf_voxel = &block_ptr->getVoxelByLinearIndex(linear_index);
      const Point t_C_voxel =
          T_G_C.inverse() *
          block_ptr->computeCoordinatesFromLinearIndex(linear_index);
      updateTsdfVoxel(range_image, color_image, t_C_voxel, tsdf_voxel,
                      deintegrate);
    }
  }
}

template <InterpolationScheme interpolation_scheme>
void ProjectiveTsdfIntegrator<interpolation_scheme>::updateTsdfVoxel(
    const Eigen::MatrixXf &range_image, ColorImage *color_image,
    const Point &t_C_voxel, TsdfVoxel *tsdf_voxel, const bool deintegrate) {
  // Skip voxels that are too far or too close
  const float distance_to_voxel = t_C_voxel.norm();
  if (distance_to_voxel < config_.min_ray_length_m ||
      distance_to_voxel > config_.max_ray_length_m) {
    return;
  }

  // Project the current voxel into the range image
  float h, w;
  if (!bearingToImage(t_C_voxel / distance_to_voxel, &h, &w)) {
    return;
  }

  // Compute the signed distance
  const float distance_to_surface = interpolate(range_image, h, w);
  const float sdf = distance_to_surface - distance_to_voxel;

  // Approximate how many rays would have updated the voxel
  // NOTE: We do this to reflect that we are more certain about
  //       the updates applied to nearer voxels
  const float num_rays_intersecting_voxel =
      ray_intersections_per_distance_squared_ /
      std::max(distance_to_voxel * distance_to_voxel, 1.f);

  // Skip voxels that fall outside the TSDF truncation distance
  if (sdf < -config_.default_truncation_distance) {
    return;
  }
  if (!config_.voxel_carving_enabled &&
      sdf > config_.default_truncation_distance) {
    return;
  }

  // Compute the weight of the measurement
  float observation_weight;
  {
    // Set the sign of the measurement weight
    if (deintegrate) {
      observation_weight = -num_rays_intersecting_voxel;
    } else {
      observation_weight = num_rays_intersecting_voxel;
    }

    // NOTE: Scaling the weight by the inverse square depth is not yet
    //       supported, since this problem is ill-defined for LiDAR

    // Apply weight drop-off if appropriate
    const FloatingPoint dropoff_epsilon = voxel_size_;
    if (config_.use_weight_dropoff && sdf < -dropoff_epsilon) {
      observation_weight *=
          (config_.default_truncation_distance + sdf) /
          (config_.default_truncation_distance - dropoff_epsilon);
    }

    // Apply sparsity compensation if appropriate
    if (config_.use_sparsity_compensation_factor) {
      if (std::abs(sdf) < config_.default_truncation_distance) {
        observation_weight *= config_.sparsity_compensation_factor;
      }
    }
  }

  // Truncate the new total voxel weight according to the max weight
  const float new_voxel_weight =
      std::min(tsdf_voxel->weight + observation_weight, config_.max_weight);

  // Make sure voxels go back to zero when deintegrating
  if (deintegrate && new_voxel_weight < 1e-3) {
    tsdf_voxel->distance = 0.0f;
    tsdf_voxel->weight = 0.0f;
    return;
  }

  if (color_image != nullptr)
    tsdf_voxel->color = color_image->operator()(std::round(h), std::round(w));
  tsdf_voxel->distance = (tsdf_voxel->distance * tsdf_voxel->weight +
                          std::min(config_.default_truncation_distance, sdf) *
                              observation_weight) /
                         new_voxel_weight;
  tsdf_voxel->weight = new_voxel_weight;
  tsdf_voxel->history.emplace(obs_cnt_);
}

template <InterpolationScheme interpolation_scheme>
template <typename T>
Point ProjectiveTsdfIntegrator<interpolation_scheme>::imageToBearing(
    const T h, const T w) const {
  double altitude_angle =
      vertical_fov_rad_ * (1.0 / 2.0 - h / (vertical_resolution_ - 1.0));
  double azimuth_angle =
      (2.0 * M_PI) * (1.0 / 2.0 - w / horizontal_resolution_);

  Point bearing;
  bearing.x() = std::cos(altitude_angle) * std::cos(azimuth_angle);
  bearing.y() = -std::cos(altitude_angle) * std::sin(azimuth_angle);
  bearing.z() = std::sin(altitude_angle);

  return bearing;
}

template <InterpolationScheme interpolation_scheme>
template <typename T>
bool ProjectiveTsdfIntegrator<interpolation_scheme>::bearingToImage(
    const Point &b_C_normalized, T *h, T *w) const {
  CHECK_NOTNULL(h);
  CHECK_NOTNULL(w);

  double altitude_angle = std::asin(b_C_normalized.z());
  // Make sure to round to nearest (not to 0) when using integers
  if (std::numeric_limits<T>::is_integer) {
    *h = std::round((vertical_resolution_ - 1.0) *
                    (1.0 / 2.0 - altitude_angle / vertical_fov_rad_));
  } else {
    *h = (vertical_resolution_ - 1.0) *
         (1.0 / 2.0 - altitude_angle / vertical_fov_rad_);
  }
  if (*h < 0.0 || vertical_resolution_ - 1.0 < *h) {
    return false;
  }

  const double azimuth_angle =
      std::atan2(-b_C_normalized.y(), b_C_normalized.x());

  // Handle integer and floating point types appropriately
  if (std::numeric_limits<T>::is_integer) {
    *w = std::round(horizontal_resolution_ *
                    (1.0 / 2.0 - azimuth_angle / (2.0 * M_PI)));
    *w = std::fmod(*w, static_cast<T>(horizontal_resolution_));
    if (*w < 0) {
      *w += horizontal_resolution_;
    }
  } else {
    *w = horizontal_resolution_ * (1.0 / 2.0 - azimuth_angle / (2.0 * M_PI));
    *w = std::fmod(*w, static_cast<T>(horizontal_resolution_));
    if (*w < 0.0) {
      // NOTE: The comparison below is a workaround to avoid the change in
      //       floating point precision around zero and horizontal_resolution_
      //       (e.g. A < 0.0 = true && A + B < B = false for A = -1e-6; B = 1e2)
      if (*w + horizontal_resolution_ < horizontal_resolution_) {
        *w += horizontal_resolution_;
      } else {
        // Negligibly small values will be truncated
        *w = 0.0;
      }
    }
  }

  return true;
}

template <>
inline float
ProjectiveTsdfIntegrator<InterpolationScheme::kNearestNeighbor>::interpolate(
    const Eigen::MatrixXf &range_image, const float h, const float w) const {
  return range_image(std::round(h), std::round(w));
}

template <>
inline float
ProjectiveTsdfIntegrator<InterpolationScheme::kMinNeighbor>::interpolate(
    const Eigen::MatrixXf &range_image, const float h, const float w) const {
  // TODO(victorr): Implement better edge handling
  if (h >= vertical_resolution_ || w >= horizontal_resolution_) {
    return range_image(std::floor(h), std::floor(w));
  }

  const float min_neighbor =
      range_image.block<2, 2>(std::floor(h), std::floor(w)).minCoeff();

  if (min_neighbor < config_.min_ray_length_m) {
    return range_image(std::round(h), std::round(w));
  } else {
    return min_neighbor;
  }
}

template <>
inline float
ProjectiveTsdfIntegrator<InterpolationScheme::kBilinear>::interpolate(
    const Eigen::MatrixXf &range_image, const float h, const float w) const {
  // TODO(victorr): Implement better edge handling
  // Check if we're on the edge, to avoid out-of-bounds matrix access
  if (h >= vertical_resolution_ || w >= horizontal_resolution_) {
    return range_image(std::floor(h), std::floor(w));
  }

  // Check if all of the neighbors are valid values
  // NOTE: This for example removes LiDAR points that had no return,
  //       which are encoded as zeros
  if ((range_image.block<2, 2>(std::floor(h), std::floor(w)).array() <
       config_.min_ray_length_m)
          .any()) {
    return range_image(std::round(h), std::round(w));
  }

  // TODO(victorr): Write this as a matrix
  float h_int, w_int;
  const float h_decimals = std::modf(h, &h_int);
  const float w_decimals = std::modf(w, &w_int);
  const float left = (1 - h_decimals) * range_image(h_int, w_int) +
                     h_decimals * range_image(h_int + 1, w_int);
  const float right = (1 - h_decimals) * range_image(h_int, w_int + 1) +
                      h_decimals * range_image(h_int + 1, w_int + 1);
  return (1 - w_decimals) * left + w_decimals * right;
}

template <>
inline float
ProjectiveTsdfIntegrator<InterpolationScheme::kAdaptive>::interpolate(
    const Eigen::MatrixXf &range_image, const float h, const float w) const {
  // TODO(victorr): Implement better horizontal wrap-around handling and
  //                out of bounds access assertions
  // Check if we're on the edge, to avoid out-of-bounds matrix access
  if (h >= vertical_resolution_ || w >= horizontal_resolution_) {
    return range_image(std::floor(h), std::floor(w));
  }

  // Get the bilinear interpolation coefficients
  float h_int, w_int;
  const float h_decimals = std::modf(h, &h_int);
  const float w_decimals = std::modf(w, &w_int);

  // Define easy to read names for all four range image pixels
  const float top_left = range_image(h_int, w_int);
  const float top_right =
      range_image(h_int, std::fmod(w_int + 1, horizontal_resolution_));
  const float bottom_left = range_image(h_int + 1, w_int);
  const float bottom_right =
      range_image(h_int + 1, std::fmod(w_int + 1, horizontal_resolution_));

  // Left column
  float bilinear_left, min_left;
  if (top_left < config_.min_ray_length_m) {
    min_left = bottom_left;
    bilinear_left = bottom_left;
  } else if (bottom_left < config_.min_ray_length_m) {
    min_left = top_left;
    bilinear_left = top_left;
  } else {
    min_left = std::min(top_left, bottom_left);
    bilinear_left = (1 - h_decimals) * top_left + h_decimals * bottom_left;
  }

  // Right column
  float bilinear_right, min_right;
  if (top_right < config_.min_ray_length_m) {
    min_right = bottom_right;
    bilinear_right = bottom_right;
  } else if (bottom_right < config_.min_ray_length_m) {
    min_right = top_right;
    bilinear_right = top_right;
  } else {
    min_right = std::min(bottom_right, top_right);
    bilinear_right = (1 - h_decimals) * top_right + h_decimals * bottom_right;
  }

  // Combine left and right bilinear values
  float bilinear_distance;
  if (bilinear_left < config_.min_ray_length_m) {
    if (bilinear_right < config_.min_ray_length_m) {
      bilinear_distance = 0.0f;
    } else {
      bilinear_distance = bilinear_right;
    }
  } else if (bilinear_right < config_.min_ray_length_m) {
    bilinear_distance = bilinear_left;
  } else {
    bilinear_distance =
        (1 - w_decimals) * bilinear_left + w_decimals * bilinear_right;
  }

  // Combine left and right min values
  float min_distance;
  if (min_left < config_.min_ray_length_m) {
    if (min_right < config_.min_ray_length_m) {
      min_distance = 0.0f;
    } else {
      min_distance = min_right;
    }
  } else if (min_right < config_.min_ray_length_m) {
    min_distance = min_left;
  } else {
    min_distance = std::min(min_left, min_right);
  }

  // Use the bilinear distance unless the min distance is much smaller,
  // in which case we go for the safe option
  if (min_distance + 0.5f * config_.default_truncation_distance <
      bilinear_distance) {
    return min_distance;
  } else {
    return bilinear_distance;
  }
}
}  // namespace voxblox

#endif  // VOXBLOX_INCLUDE_VOXBLOX_INTEGRATOR_PROJECTIVE_TSDF_INTEGRATOR_INL_H_
