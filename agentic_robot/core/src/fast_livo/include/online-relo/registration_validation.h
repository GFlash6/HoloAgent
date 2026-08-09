#pragma once

#include <cmath>

#include <Eigen/Core>

inline bool registrationWithinTranslationPrior(const Eigen::Matrix4f &initial,
                                               const Eigen::Matrix4f &result,
                                               double maximum_distance) {
  if (!initial.allFinite() || !result.allFinite() ||
      !std::isfinite(maximum_distance) || maximum_distance < 0.0) {
    return false;
  }
  const Eigen::Vector3f delta =
      result.block<3, 1>(0, 3) - initial.block<3, 1>(0, 3);
  return static_cast<double>(delta.norm()) <= maximum_distance;
}
