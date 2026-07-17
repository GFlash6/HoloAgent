python3 compare_cpu_mem.py ${HOLOAGENT_DATA_ROOT}/VLN/Log/PCL_NDT_0.05.txt \
 ${HOLOAGENT_DATA_ROOT}/VLN/Log/PCL_ICP_0.05.txt \
 ${HOLOAGENT_DATA_ROOT}/VLN/Log/GPU_NDT_0.05.txt \
 -o ${HOLOAGENT_DATA_ROOT}/VLN/Log/compare_0.05.png --no-show

# python3 compare_cpu_mem.py ${HOLOAGENT_DATA_ROOT}/VLN/Log/PCL_NDT_0.05_DENSE.txt \
#  ${HOLOAGENT_DATA_ROOT}/VLN/Log/PCL_ICP_0.05_DENSE.txt \
#  ${HOLOAGENT_DATA_ROOT}/VLN/Log/GPU_NDT_0.05_DENSE.txt \
#  -o ${HOLOAGENT_DATA_ROOT}/VLN/Log/compare_0.05_dense.png --no-show