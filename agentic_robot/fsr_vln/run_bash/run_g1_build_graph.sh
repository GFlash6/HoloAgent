
all_scenes=("sh_3f","ic_4f")
# all_scenes=("ic_7f","ic_3f")
all_scenes=("ic_3f")

cd ../
CUDA_VISIBLE_DEVICES=2,3,4,5,6,7 python run_holoagent_mapping.py \
 --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
 --dataset_name G1 \
 --experiment_name holoagent_mapping-g1\
 --scenes_list "$(IFS=, ; echo "${all_scenes[*]}")" \
 --build_hmsg \
 --hmsg_config_path ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/holoagent_mapping/mapvln/ovo_slam/configs/hmsg_bridge.yaml \
 --cfg_path ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/holoagent_mapping/mapvln/ovo_slam/configs/ovo_g1.yaml \

# pkill -9 -f run_holoagent_mapping.py
