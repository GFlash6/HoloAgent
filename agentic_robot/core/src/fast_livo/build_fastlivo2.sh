# 3. 设置环境
export PATH=/usr/bin:$PATH
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages
unset PYTHONHOME
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH

# 4. Source ROS
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_livo   --symlink-install   --event-handlers console_direct+   --parallel-workers 1   --cmake-args   -DCURL_LIBRARY=/usr/lib/x86_64-linux-gnu/libcurl.so.4   -DPYTHON_EXECUTABLE=/usr/bin/python3.10   -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM   -DGTSAM_ROOT_DIR=/usr/local   -DGTSAM_LIBRARY=/usr/local/lib/libgtsam.so   -DCMAKE_PREFIX_PATH="/usr/local;$CMAKE_PREFIX_PATH"   -DCMAKE_INSTALL_RPATH="/usr/local/lib"   -DCMAKE_BUILD_RPATH="/usr/local/lib"
