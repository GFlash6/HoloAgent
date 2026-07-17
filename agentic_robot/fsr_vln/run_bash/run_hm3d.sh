
# all_scenes=("00813-svBbv1Pavdk" "00814-p53SfW6mjZe" 
# "00815-h1zeeAwLh9Z" "00820-mL8ThkuaVTM" "00821-eF36g7L6Z9M" "00823-7MXmsvcQjpJ" 
# "00824-Dd4bFSTQ8gi" "00827-BAbdmeyTvMZ" "00829-QaLdnwvtxbs" 
# "00831-yr17PDCnDDW" "00832-qyAac8rV8Zk")
all_scenes=("00803-k1cupFYWXJ6")
cd ../
CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python run_holoagent_mapping.py \
 --data_dir ${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/data/ \
 --dataset_name HM3D \
 --experiment_name holoagent_mapping-hm3d\
 --scenes_list "$(IFS=, ; echo "${all_scenes[*]}")" \
 --run \
 --hmsg_config_path configs/hmsg_bridge_hm3d.yaml \
 --cfg_path configs/ovo_hm3d.yaml

pkill -9 -f run_holoagent_mapping.py
