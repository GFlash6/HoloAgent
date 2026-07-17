
# CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_ovo_mapping.py --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
#  --dataset_name G1 \
#  --experiment_name ovo_mapping \
#  --scenes_list sh_3f \
#  --run 
# 运行完成后kill掉进程
#"scene0011_00", "scene0050_00","scene0231_00","scene0378_00", "scene0518_00"
# all_scenes=("scene0011_00", "scene0050_00" "scene0231_00" "scene0378_00" "scene0518_00")
all_scenes=("scene0518_00")

# all_scenes=("scene0050_00")
CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_ovo_mapping.py \
 --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
 --dataset_name ScanNet \
 --experiment_name ovo_mapping_test \
 --scenes_list "$(IFS=, ; echo "${all_scenes[*]}")" \
 --segment \
 --eval \
 --run \
 --cfg_path ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/mapvln/ovo_slam/configs/ovo_new.yaml \
 --dataset_info_file ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/mapvln/ovo_slam/configs/ScanNet/eval_info.yaml
pkill -9 -f run_ovo_mapping.py
