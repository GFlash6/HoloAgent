export CUDA_VISIBLE_DEVICES=2 
python visualize_scene.py --run_path \
 ${HOLOAGENT_DATA_ROOT}/VLN/OVO/data/output/ScanNet/ovo_mapping/rosbag2_2025_04_08-21_33_17_fast_livo \
 --visualize_obj --visualize_interactive_query