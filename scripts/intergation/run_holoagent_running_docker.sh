#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

terminator &
sleep 0.6

# 聚焦 terminator 窗口
xdotool search --class terminator windowactivate --sync %%

# 左上 pane：先启动容器，后续 pane 复用
xdotool type "1"
xdotool key Return
xdotool type "docker start holoagent_running"
xdotool key Return
xdotool type "xhost +"
xdotool key Return
sleep 0.5

# 左右分屏
xdotool key ctrl+shift+e
sleep 0.2

# 左侧再上下分屏
xdotool key alt+Left
sleep 0.2
xdotool key ctrl+shift+o
sleep 0.2

# 右侧再上下分屏
xdotool key alt+Right
sleep 0.2
xdotool key ctrl+shift+o
sleep 0.2

# 左上 pane：启动定位
xdotool key alt+Left alt+Up
sleep 0.2
xdotool type "1"
xdotool key Return
xdotool type "docker exec -it holoagent_running /bin/bash"
xdotool key Return
xdotool type "cd ${REPO_ROOT}"
xdotool key Return
xdotool type "bash robots/unitree/scripts/run_localization.sh"
xdotool key Return

# 左下 pane：启动导航
xdotool key alt+Left alt+Down
sleep 0.2
xdotool type "1"
xdotool key Return
xdotool type "docker exec -it holoagent_running /bin/bash"
xdotool key Return
xdotool type "cd ${REPO_ROOT}"
xdotool key Return
xdotool type "bash robots/unitree/scripts/run_nav.sh"
xdotool key Return

# 右上 pane：启动语义导航
xdotool key alt+Right alt+Up
sleep 0.2
xdotool type "1"
xdotool key Return
xdotool type "docker exec -it holoagent_running /bin/bash"
xdotool key Return
xdotool type "cd ${REPO_ROOT}"
xdotool key Return
xdotool type "bash robots/unitree/scripts/run_sem_nav.sh"
xdotool key Return

# 右下 pane：启动 sensor_ctl
xdotool key alt+Right alt+Down
sleep 0.2
xdotool type "1"
xdotool key Return
xdotool type "docker exec -it holoagent_running /bin/bash"
xdotool key Return
xdotool type "cd ${REPO_ROOT}"
xdotool key Return
xdotool type "bash robots/unitree/scripts/run_sensor_ctl.sh"
xdotool key Return