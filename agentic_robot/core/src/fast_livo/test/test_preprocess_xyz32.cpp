#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "preprocess.h"

TEST(PreprocessXYZ32, PreservesFiniteGeometryWithoutColorFields) {
  pcl::PointCloud<pcl::PointXYZ> input;
  input.emplace_back(1.0F, 2.0F, 3.0F);
  input.emplace_back(4.0F, 5.0F, 6.0F);
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(input, message);

  Preprocess preprocess;
  preprocess.lidar_type = XYZ32;
  preprocess.blind = 0.1;
  preprocess.max_range = 100.0;
  preprocess.point_filter_num = 1;
  PointCloudXYZI::Ptr output(new PointCloudXYZI());
  preprocess.process(
      std::make_shared<const sensor_msgs::msg::PointCloud2>(message), output);

  ASSERT_EQ(output->size(), 2U);
  EXPECT_FLOAT_EQ(output->points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(output->points[0].y, 2.0F);
  EXPECT_FLOAT_EQ(output->points[0].z, 3.0F);
  EXPECT_FLOAT_EQ(output->points[0].intensity, 0.0F);
}
