
# CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_ovo_mapping.py --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
#  --dataset_name G1 \
#  --experiment_name ovo_mapping \
#  --scenes_list sh_3f \
#  --run 
# 运行完成后kill掉进程
CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_ovo_mapping.py --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data \
 --dataset_name ScanNet \
 --experiment_name ovo_mapping \
 --scenes_list scene0000_00 \
 --eval
pkill -9 -f run_ovo_mapping.py
# CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_eval.py --dataset_name G1 \
#  --experiment_name ovo_mapping --scenes_list sh_3f,ic_3f,ic_4f,ic_7f  \
#  --run
# CUDA_VISIBLE_DEVICES=1 python run_eval.py --dataset_name G1 \
#  --experiment_name ovo_mapping --scenes_list ic_3f  \
#  --run