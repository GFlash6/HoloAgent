#!/usr/bin/env bash
#
# Clear build artifacts for HoloAgent ROS 2 workspaces.
#
# Usage:
#   bash agentic_robot/clear.sh
#   bash agentic_robot/clear.sh -w all
#   bash agentic_robot/clear.sh -w thirdparty
#   bash agentic_robot/clear.sh -w core
#   bash agentic_robot/clear.sh -w services
#   bash agentic_robot/clear.sh -w robots
#   bash agentic_robot/clear.sh --dry-run
#
# Removed directories in each selected workspace:
#   build/ install/ log/
# Also removes:
#   all __pycache__ directories under agentic_robot/
#   build/ install/ log/ under robots/*/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET_WS="all"
DRY_RUN=0

workspace_root() {
    case "$1" in
        thirdparty) echo "$SCRIPT_DIR/thirdparty" ;;
        core) echo "$SCRIPT_DIR/core" ;;
        services) echo "$SCRIPT_DIR/services" ;;
        robots) echo "$REPO_ROOT/robots" ;;
        *)
            echo "[ERROR] unknown workspace: $1" >&2
            echo "[ERROR] valid workspaces: all | thirdparty | core | services | robots" >&2
            exit 1
            ;;
    esac
}

usage() {
    sed -n '3,14p' "$0" | sed 's/^# \{0,1\}//'
}

remove_dir() {
    local dir="$1"

    if [ "$DRY_RUN" -eq 1 ]; then
        echo "[DRY-RUN] rm -rf $dir"
        return 0
    fi

    if [ -e "$dir" ]; then
        rm -rf "$dir"
        echo "[REMOVED] $dir"
    else
        echo "[SKIP] not found: $dir"
    fi
}

clear_pycache() {
    echo "[CLEAR] python cache: $SCRIPT_DIR"

    while IFS= read -r -d '' pycache_dir; do
        remove_dir "$pycache_dir"
    done < <(find "$SCRIPT_DIR" -type d -name __pycache__ -print0)
}

clear_workspace() {
    local ws="$1"
    local root

    root="$(workspace_root "$ws")"

    if [ ! -d "$root" ]; then
        echo "[ERROR] workspace directory does not exist: $root" >&2
        exit 1
    fi

    echo "[CLEAR] workspace: $ws"
    echo "[CLEAR] root     : $root"

    remove_dir "$root/build"
    remove_dir "$root/install"
    remove_dir "$root/log"
}

clear_robot_subworkspaces() {
    local robots_root="$REPO_ROOT/robots"
    local robot_dir

    if [ ! -d "$robots_root" ]; then
        echo "[SKIP] robots root not found: $robots_root"
        return 0
    fi

    echo "[CLEAR] robot sub-workspaces under: $robots_root"

    for robot_dir in "$robots_root"/*; do
        if [ -d "$robot_dir" ]; then
            remove_dir "$robot_dir/build"
            remove_dir "$robot_dir/install"
            remove_dir "$robot_dir/log"
        fi
    done
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        -w|--workspace)
            TARGET_WS="${2:-}"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

case "$TARGET_WS" in
    all)
        clear_workspace thirdparty
        clear_workspace core
        clear_workspace services
        clear_workspace robots
        clear_robot_subworkspaces
        ;;
    thirdparty|core|services)
        clear_workspace "$TARGET_WS"
        ;;
    robots)
        clear_workspace robots
        clear_robot_subworkspaces
        ;;
    *)
        workspace_root "$TARGET_WS" >/dev/null
        ;;
esac

clear_pycache

echo "[DONE]"