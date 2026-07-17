#!/usr/bin/env bash
#
# Build the Unitree G1 robot-specific ROS 2 workspace.
#
# Usage:
#   bash robots/g1/build.sh
#   bash robots/g1/build.sh -p g1_move
#   bash robots/g1/build.sh -p "g1_move g1_arm"
#   bash robots/g1/build.sh -j 2
#   bash robots/g1/build.sh --dry-run

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROBOT_NAME="g1"

COLCON_ARGS=(--symlink-install)
TARGET_PKGS=()
JOBS=""
DRY_RUN=0

usage() {
    sed -n '3,11p' "$0" | sed 's/^# \{0,1\}//'
}

run_cmd() {
    if [ "$DRY_RUN" -eq 1 ]; then
        printf '[DRY-RUN]'
        printf ' %q' "$@"
        printf '\n'
        return 0
    fi

    "$@"
}

source_setup() {
    local setup_file="$1"

    if [ "$DRY_RUN" -eq 1 ]; then
        echo "[DRY-RUN] source $setup_file"
        return 0
    fi

    if [ -f "$setup_file" ]; then
        set +u
        # shellcheck disable=SC1090
        source "$setup_file"
        set -u
        echo "[SOURCE] $setup_file"
    else
        echo "[WARN] underlay not built yet: $setup_file"
    fi
}

source_agentic_underlay() {
    source_setup "$REPO_ROOT/agentic_robot/thirdparty/install/setup.bash"
    source_setup "$REPO_ROOT/agentic_robot/core/install/setup.bash"
    source_setup "$REPO_ROOT/agentic_robot/services/install/setup.bash"
}

parse_package_list() {
    local value="$1"
    local pkg
    for pkg in $value; do
        TARGET_PKGS+=("$pkg")
    done
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        -p|--package|--packages)
            parse_package_list "${2:-}"
            shift 2
            ;;
        -j|--jobs)
            JOBS="${2:-}"
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

if [ -n "$JOBS" ] && ! [[ "$JOBS" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] --jobs must be a positive integer" >&2
    exit 1
fi

source_agentic_underlay

cmd=(colcon build "${COLCON_ARGS[@]}" --base-paths src)
if [ -n "$JOBS" ]; then
    cmd+=(--parallel-workers "$JOBS")
fi
if [ "${#TARGET_PKGS[@]}" -gt 0 ]; then
    cmd+=(--packages-select "${TARGET_PKGS[@]}")
fi

echo "[BUILD] layer    : robots/$ROBOT_NAME"
echo "[BUILD] root     : $SCRIPT_DIR"
if [ "${#TARGET_PKGS[@]}" -gt 0 ]; then
    echo "[BUILD] packages : ${TARGET_PKGS[*]}"
else
    echo "[BUILD] packages : all"
fi

cd "$SCRIPT_DIR"
if [ -n "$JOBS" ]; then
    MAKEFLAGS="-j${JOBS}" run_cmd "${cmd[@]}"
else
    run_cmd "${cmd[@]}"
fi

echo "[DONE]"
