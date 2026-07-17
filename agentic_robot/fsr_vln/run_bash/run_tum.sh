
# CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_ovo_mapping.py --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
#  --dataset_name G1 \
#  --experiment_name ovo_mapping \
#  --scenes_list sh_3f \
#  --run 
# 运行完成后kill掉进程
#"scene0011_00", "scene0050_00","scene0231_00","scene0378_00", "scene0518_00"
all_scenes=("rgbd_dataset_freiburg1_360" "rgbd_dataset_freiburg1_desk" "rgbd_dataset_freiburg1_desk2" 
"rgbd_dataset_freiburg1_floor" "rgbd_dataset_freiburg1_plant")

all_scenes=("rgbd_dataset_freiburg1_teddy" "rgbd_dataset_freiburg1_xyz")
all_scenes=("rgbd_dataset_freiburg1_room")
# all_scenes=("scene0050_00")
# CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_ovo_mapping.py \
#  --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
#  --dataset_name TUM \
#  --experiment_name tum_mapping_test_iggt_loop \
#  --scenes_list "$(IFS=, ; echo "${all_scenes[*]}")" \
#  --run \
#  --eval_trajectory \
#  --cfg_path ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/mapvln/ovo_slam/configs/ovo_tum.yaml \
#  --dataset_info_file ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/mapvln/ovo_slam/configs/ScanNet/eval_info.yaml
cd ../
CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 \
exec -a tum_ovo_mapping \
python run_ovo_mapping.py \
 --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
 --dataset_name TUM \
 --experiment_name tum_mapping_test_iggt_loop_tum_demo_new \
 --scenes_list "$(IFS=, ; echo "${all_scenes[*]}")" \
 --run \
 --eval_trajectory \
 --cfg_path ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/mapvln/ovo_slam/configs/ovo_tum.yaml \
 --dataset_info_file ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/mapvln/ovo_slam/configs/ScanNet/eval_info.yaml
pkill -9 -f tum_ovo_mapping