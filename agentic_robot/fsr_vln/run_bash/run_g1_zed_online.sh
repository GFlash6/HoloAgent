
# cd ../
CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_stream_mapping.py \
 --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
 --scene_name g1_zed \
 --dataset_name Zed \
 --experiment_name g1_zed  \
 --config_path configs/ovo_zed.yaml 
