
# all_scenes=("ic_3f","ic_7f","ic_4f")
# rosbag2_2024_12_06-10_35_09
# all_scenes=("rosbag2_2024_12_06-10_35_09")
#rosbag2_2024_12_24-18_41_50
all_scenes=("rosbag2_2024_12_24-18_41_50")
cd ../
CUDA_VISIBLE_DEVICES=2,3,4,5,6,7 python run_ovo_mapping.py \
 --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
 --dataset_name Go2 \
 --experiment_name ovo_mapping_go2\
 --scenes_list "$(IFS=, ; echo "${all_scenes[*]}")" \
 --run \
 --cfg_path ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/mapvln/ovo_slam/configs/ovo_g1_pred.yaml 

pkill -9 -f run_ovo_mapping.py
pkill -9 -f monitor.py