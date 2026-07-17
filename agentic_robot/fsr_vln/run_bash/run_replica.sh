# CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_ovo_mapping.py --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
#  --dataset_name G1 \
#  --experiment_name ovo_mapping \
#  --scenes_list sh_3f \
#  --run 
# 运行完成后kill掉进程
#"scene0011_00", "scene0050_00","scene0231_00","scene0378_00", "scene0518_00"
all_scenes=("office0" "office1" "office2" "office3" "office4" "room0" "room1" "room2")
# all_scenes=("office2" "office3" "office4" "room0" "room1" "room2")
# all_scenes=("office0")

cd ../
CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_ovo_mapping_replica.py \
 --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
 --dataset_name Replica \
 --experiment_name ovo_mapping_replica_pred_2 \
 --scenes_list "$(IFS=, ; echo "${all_scenes[*]}")" \
 --run \
 --cfg_path ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/mapvln/ovo_slam/configs/ovo_replica_pred.yaml \
 --dataset_info_file ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/mapvln/ovo_slam/configs/Replica/eval_info.yaml
pkill -9 -f run_ovo_mapping_replica.py
# monitor.py
pkill -9 -f monitor.py
