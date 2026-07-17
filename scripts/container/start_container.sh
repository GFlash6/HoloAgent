#!/bin/bash

# docker kill holoagent_orin_deploy 
# docker rm holoagent_orin_deploy 
# echo "Old holoagent_orin_deploy container removed !"

sudo docker run -it \
  --name holoagent_running \
  --runtime nvidia \
  --gpus all \
  --privileged \
  --network host \
  -e "ACCEPT_EULA=Y" \
  -e DISPLAY=$DISPLAY \
  -v /home/unitree/agentic_robot_system:/workspace \
  -v /dev:/dev \
  -v /usr/local/cuda:/usr/local/cuda \
  -v /lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "/home/unitree/agentic_robot_system/holoagent/holomotion_repo/HoloMotion:/home/unitree/holomotion" \
  -v "/usr/local/cuda-11.4/targets/aarch64-linux/lib:/cuda_base:ro" \
  -v "/usr/lib/aarch64-linux-gnu/libcudnn.so.8.6.0:/host_gpu/libcudnn.so.8.6.0:ro" \
  -v "/usr/lib/aarch64-linux-gnu/libcudnn_ops_infer.so.8.6.0:/host_gpu/libcudnn_ops_infer.so.8.6.0:ro" \
  -v "/usr/lib/aarch64-linux-gnu/libcudnn_cnn_infer.so.8.6.0:/host_gpu/libcudnn_cnn_infer.so.8.6.0:ro" \
  holoagent_running:latest
  bash -c "ln -sf /host_gpu/libcudnn.so.8.6.0 /host_gpu/libcudnn.so.8 && \
           ln -sf /host_gpu/libcudnn_ops_infer.so.8.6.0 /host_gpu/libcudnn_ops_infer.so.8 && \
           ln -sf /host_gpu/libcudnn_cnn_infer.so.8.6.0 /host_gpu/libcudnn_cnn_infer.so.8 && \
           source /root/miniconda3/bin/activate && conda activate holomotion_deploy && exec bash"
