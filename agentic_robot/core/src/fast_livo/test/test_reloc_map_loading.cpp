#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

#include "multi-session/Incremental_mapping.hpp"

namespace fs = std::filesystem;

namespace {

class TempRelocMap {
 public:
  TempRelocMap() {
    path = fs::temp_directory_path() /
           ("fast_livo_reloc_map_" + std::to_string(::getpid()));
    fs::create_directories(path / "keyframe_cloud");
    fs::create_directories(path / "keyframe_scancontext");

    write(path / "singlesession_posegraph.g2o",
          "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\n");
    const std::string pcd =
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH 2\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS 2\n"
        "DATA ascii\n"
        "1 2 3 4\n"
        "-1 -2 -3 5\n";
    write(path / "cloudGlobal.pcd", pcd);
    write(path / "keyframe_cloud" / "000000.pcd", pcd);

    std::ofstream scd(path / "keyframe_scancontext" / "000000.scd");
    for (int row = 0; row < 40; ++row) {
      for (int col = 0; col < 120; ++col) {
        scd << 0.0 << (col == 119 ? '\n' : ' ');
      }
    }
  }

  ~TempRelocMap() { fs::remove_all(path); }

  fs::path path;

 private:
  static void write(const fs::path &target, const std::string &contents) {
    std::ofstream stream(target);
    stream << contents;
  }
};

TEST(RelocMapLoading, CopiesPointsReadFromPcdIntoKeyframe) {
  TempRelocMap map;
  MultiSession::Session session(1, "test", map.path.string(), true);

  ASSERT_EQ(session.cloudKeyFrames.size(), 1U);
  ASSERT_NE(session.cloudKeyFrames[0].all_cloud, nullptr);
  ASSERT_EQ(session.cloudKeyFrames[0].all_cloud->size(), 2U);
  EXPECT_FLOAT_EQ(session.cloudKeyFrames[0].all_cloud->points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(session.cloudKeyFrames[0].all_cloud->points[0].y, 2.0F);
  EXPECT_FLOAT_EQ(session.cloudKeyFrames[0].all_cloud->points[0].z, 3.0F);
  EXPECT_FLOAT_EQ(session.cloudKeyFrames[0].all_cloud->points[0].intensity,
                  4.0F);
}

}  // namespace
