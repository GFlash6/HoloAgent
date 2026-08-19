#!/usr/bin/env bash
#
# Build non-robot HoloAgent ROS 2 workspaces.
#
# Usage:
#   bash agentic_robot/build.sh
#   bash agentic_robot/build.sh -w all
#   bash agentic_robot/build.sh -w thirdparty
#   bash agentic_robot/build.sh -w core
#   bash agentic_robot/build.sh -w services
#   bash agentic_robot/build.sh -p nav_executor
#   bash agentic_robot/build.sh -w core -p "nav_executor relative_goal"
#   bash agentic_robot/build.sh -w core -j 2
#   bash agentic_robot/build.sh -w all --dry-run
#
# Workspace order:
#   thirdparty -> core -> services

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS Humble is installed for Ubuntu's Python 3.10.  Pin CMake interface
# generation so an activated/newer conda Python cannot produce unloadable
# rosidl type-support extensions.
COLCON_ARGS=(
    --symlink-install
    --cmake-args
    -DPython3_EXECUTABLE=/usr/bin/python3
    -DPYTHON_EXECUTABLE=/usr/bin/python3
)
TARGET_WS="all"
TARGET_PKGS=()
JOBS=""
DRY_RUN=0

workspace_root() {
    case "$1" in
        thirdparty) echo "$SCRIPT_DIR/thirdparty" ;;
        core) echo "$SCRIPT_DIR/core" ;;
        services) echo "$SCRIPT_DIR/services" ;;
        *)
            echo "[ERROR] unknown agentic workspace: $1" >&2
            echo "[ERROR] valid workspaces: all | thirdparty | core | services" >&2
            exit 1
            ;;
    esac
}

workspace_base_path() {
    case "$1" in
        thirdparty) echo "src" ;;
        core) echo "src" ;;
        services) echo "src" ;;
        *) workspace_root "$1" >/dev/null ;;
    esac
}

workspace_dependencies() {
    case "$1" in
        thirdparty) echo "" ;;
        core) echo "thirdparty" ;;
        services) echo "thirdparty core" ;;
        *) workspace_root "$1" >/dev/null ;;
    esac
}

usage() {
    sed -n '3,17p' "$0" | sed 's/^# \{0,1\}//'
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

source_workspace() {
    local ws="$1"
    local setup_file
    setup_file="$(workspace_root "$ws")/install/setup.bash"

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

source_dependencies() {
    local ws="$1"
    local dep
    for dep in $(workspace_dependencies "$ws"); do
        source_workspace "$dep"
    done
}

build_workspace() {
    local ws="$1"
    shift
    local packages=("$@")
    local root
    local base_path

    root="$(workspace_root "$ws")"
    base_path="$(workspace_base_path "$ws")"

    if [ ! -d "$root" ]; then
        echo "[ERROR] workspace directory does not exist: $root" >&2
        exit 1
    fi

    source_dependencies "$ws"

    local cmd=(colcon build "${COLCON_ARGS[@]}" --base-paths "$base_path")
    if [ -n "$JOBS" ]; then
        cmd+=(--parallel-workers "$JOBS")
    fi
    if [ "${#packages[@]}" -gt 0 ]; then
        cmd+=(--packages-select "${packages[@]}")
    fi

    echo "[BUILD] layer    : agentic_robot"
    echo "[BUILD] workspace: $ws"
    echo "[BUILD] root     : $root"
    if [ "${#packages[@]}" -gt 0 ]; then
        echo "[BUILD] packages : ${packages[*]}"
    else
        echo "[BUILD] packages : all"
    fi

    cd "$root"
    if [ -n "$JOBS" ]; then
        MAKEFLAGS="-j${JOBS}" run_cmd "${cmd[@]}"
    else
        run_cmd "${cmd[@]}"
    fi
}

package_in_workspace() {
    local ws="$1"
    local pkg="$2"
    local root
    local base_path

    root="$(workspace_root "$ws")"
    base_path="$(workspace_base_path "$ws")"

    find "$root/$base_path" -maxdepth 8 -name package.xml -print0 2>/dev/null \
        | xargs -0 grep -l "<name>${pkg}</name>" 2>/dev/null \
        | grep -q .
}

find_workspace_for_package() {
    local pkg="$1"
    local ws
    for ws in thirdparty core services; do
        if package_in_workspace "$ws" "$pkg"; then
            echo "$ws"
            return 0
        fi
    done
    return 1
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
        -w|--workspace)
            TARGET_WS="${2:-}"
            shift 2
            ;;
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

if [ "${#TARGET_PKGS[@]}" -gt 0 ] && [ "$TARGET_WS" = "all" ]; then
    if [ "${#TARGET_PKGS[@]}" -ne 1 ]; then
        echo "[ERROR] package auto-detection supports one package at a time; pass --workspace for multiple packages" >&2
        exit 1
    fi

    if ! TARGET_WS="$(find_workspace_for_package "${TARGET_PKGS[0]}")"; then
        echo "[ERROR] package not found in agentic_robot workspaces: ${TARGET_PKGS[0]}" >&2
        exit 1
    fi
fi

case "$TARGET_WS" in
    all)
        build_workspace thirdparty
        build_workspace core
        build_workspace services
        ;;
    thirdparty|core|services)
        build_workspace "$TARGET_WS" "${TARGET_PKGS[@]}"
        ;;
    *)
        workspace_root "$TARGET_WS" >/dev/null
        ;;
esac

echo "[DONE]"
