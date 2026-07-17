#!/usr/bin/env bash

set -e

# tmux sessions used by current launcher scripts:
# - robots/unitree/scripts/run_sensors.sh          -> robot_sensors
# - robots/unitree/scripts/run_localization.sh     -> robot_localization
# - robots/unitree/scripts/run_nav.sh              -> robot_nav
# - robots/unitree/scripts/run_audio.sh            -> robot_audio
# - robots/unitree/scripts/run_ctl.sh              -> robot_ctl
# - robots/unitree/scripts/run_sem_nav.sh          -> robot_sem_nav
# - robots/unitree/scripts/run_armctl.sh           -> robot_armctl
# - robots/unitree/scripts/run_velctl.sh           -> robot_velctl
# - robots/unitree/scripts/run_nav_bridge.sh       -> robot_nav_bridge
# - robots/unitree/scripts/run_sensor_ctl.sh       -> robot_nav_ctl
# - robots/hexfellow/scripts/start_all.sh          -> hexfellow

SESSIONS=(
    robot_sensors
    robot_localization
    robot_nav
    robot_audio
    robot_ctl
    robot_sem_nav
    robot_armctl
    robot_velctl
    robot_nav_bridge
    robot_nav_ctl
    hexfellow
)

for session_name in "${SESSIONS[@]}"; do
    if tmux has-session -t "${session_name}" 2>/dev/null; then
        tmux kill-session -t "${session_name}"
        echo "Killed tmux session: ${session_name}"
        sleep 0.1
    fi
done