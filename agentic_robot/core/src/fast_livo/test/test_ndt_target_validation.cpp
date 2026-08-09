#include <gtest/gtest.h>

#include "online-relo/ndt_target_validation.h"
#include "online-relo/registration_validation.h"

namespace {

using Point = pcl::PointXYZINormal;

TEST(NdtTargetValidation, RejectsCloudWithNoCovarianceVoxels) {
  auto cloud = std::make_shared<pcl::PointCloud<Point>>();
  for (int i = 0; i < 20; ++i) {
    Point point;
    point.x = static_cast<float>(i);
    point.y = 0.0f;
    point.z = 0.0f;
    cloud->push_back(point);
  }

  EXPECT_EQ(countNdtTargetVoxels<Point>(cloud, 0.2f), 0U);
}

TEST(NdtTargetValidation, AcceptsCloudWithPopulatedCovarianceVoxels) {
  auto cloud = std::make_shared<pcl::PointCloud<Point>>();
  for (int voxel = 0; voxel < 4; ++voxel) {
    for (int i = 0; i < 8; ++i) {
      Point point;
      point.x = static_cast<float>(voxel) + 0.01f * static_cast<float>(i);
      point.y = 0.01f * static_cast<float>(i % 3);
      point.z = 0.01f * static_cast<float>((i * 2) % 5);
      cloud->push_back(point);
    }
  }

  EXPECT_GE(countNdtTargetVoxels<Point>(cloud, 1.0f), 3U);
}

TEST(RegistrationValidation, RejectsLowScoreSolutionFarFromPrior) {
  Eigen::Matrix4f initial = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f wrong_symmetric_match = Eigen::Matrix4f::Identity();
  wrong_symmetric_match(1, 3) = 2.61f;

  EXPECT_FALSE(registrationWithinTranslationPrior(initial, wrong_symmetric_match, 2.0));
  EXPECT_TRUE(registrationWithinTranslationPrior(initial, wrong_symmetric_match, 3.0));
}

}  // namespace
