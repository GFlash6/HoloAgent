
all_scenes=("sh3f_s9000")
cd ../
# note the camera pose coordinate system in holoagent_mapping. 
CUDA_VISIBLE_DEVICES=2,3,4,5,6,7 python run_holoagent_mapping.py \
 --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data/ \
 --dataset_name Pocket \
 --experiment_name holoagent_mapping-pocket\
 --scenes_list "$(IFS=, ; echo "${all_scenes[*]}")" \
 --run \
 --build_hmsg \
 --hmsg_config_path configs/hmsg_bridge_sh3f_s9000.yaml \
 --cfg_path configs/ovo_pocket.yaml

pkill -9 -f run_holoagent_mapping.py
