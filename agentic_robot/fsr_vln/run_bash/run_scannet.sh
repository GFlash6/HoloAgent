
# all_scenes=("scene0000_00")
# all_scenes=("scene0378_00")
# all_scenes=("scene0000_00" "scene0059_00" "scene0106_00" "scene0169_00" "scene0181_00" "scene0207_00")
all_scenes=("scene0050_00" "scene0231_00" "scene0378_00" "scene0518_00")
cd ../
CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_holoagent_mapping.py \
 --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
 --dataset_name ScanNet \
 --experiment_name holoagent_mapping-scannet \
 --scenes_list "$(IFS=, ; echo "${all_scenes[*]}")" \
 --run \
 --build_hmsg \
 --hmsg_config_path ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/holoagent_mapping/mapvln/ovo_slam/configs/hmsg_bridge.yaml \
 --cfg_path ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/holoagent_mapping/mapvln/ovo_slam/configs/ovo_scannet.yaml \
 --dataset_info_file ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/holoagent_mapping/mapvln/ovo_slam/configs/ScanNet/eval_info.yaml
pkill -9 -f run_holoagent_mapping.py
