#include <gtest/gtest.h>

#include <Eigen/Core>

TEST(EigenPclAlignment, MatchesSystemPclAbi) {
  EXPECT_EQ(EIGEN_MAX_ALIGN_BYTES, 16)
      << "FAST-LIVO and the binary PCL packages must use the same Eigen "
         "alignment ABI";
  EXPECT_EQ(EIGEN_DEFAULT_ALIGN_BYTES, 16)
      << "CPU tuning flags must not raise Eigen's dynamic alignment above "
         "the binary PCL ABI";
  EXPECT_EQ(EIGEN_MALLOC_ALREADY_ALIGNED, 1)
      << "PCL-allocated vectors must be released with the system allocator";
}
