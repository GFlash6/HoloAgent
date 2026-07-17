#!/usr/bin/env bash
#
# Compatibility dispatcher for HoloAgent builds.
#
# Prefer the layer-specific entrypoints:
#   bash agentic_robot/build.sh
#   bash robots/g1/build.sh
#   bash robots/hexfellow/build.sh
#
# Legacy examples still supported:
#   bash scripts/build.sh -w all
#   bash scripts/build.sh -w core -p perception
#   bash scripts/build.sh -w g1
#   bash scripts/build.sh -p robot_bridge
#   bash scripts/build.sh -p <pkg>

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

TARGET_WS="all"
TARGET_PKGS=()
FORWARD_ARGS=()

usage() {
    sed -n '3,16p' "$0" | sed 's/^# \{0,1\}//'
}

parse_package_list() {
    local value="$1"
    local pkg
    for pkg in $value; do
        TARGET_PKGS+=("$pkg")
    done
}

package_exists_under() {
    local root="$1"
    local pkg="$2"

    find "$root" -maxdepth 8 -name package.xml -print0 2>/dev/null \
        | xargs -0 grep -l "<name>${pkg}</name>" 2>/dev/null \
        | grep -q .
}

detect_package_layer() {
    local pkg="$1"

    if package_exists_under "$REPO_ROOT/agentic_robot/thirdparty/src" "$pkg" \
        || package_exists_under "$REPO_ROOT/agentic_robot/core/src" "$pkg" \
        || package_exists_under "$REPO_ROOT/agentic_robot/services/src" "$pkg"; then
        echo "agentic"
        return 0
    fi

    if package_exists_under "$REPO_ROOT/robots/g1/src" "$pkg"; then
        echo "g1"
        return 0
    fi

    if package_exists_under "$REPO_ROOT/robots/hexfellow/src" "$pkg"; then
        echo "hexfellow"
        return 0
    fi

    return 1
}

run_agentic() {
    bash "$REPO_ROOT/agentic_robot/build.sh" "$@"
}

run_robot() {
    local robot="$1"
    shift
    bash "$REPO_ROOT/robots/$robot/build.sh" "$@"
}

robot_forward_args() {
    local args=()
    local skip_next=0
    local arg

    for arg in "$@"; do
        if [ "$skip_next" -eq 1 ]; then
            skip_next=0
            continue
        fi
        case "$arg" in
            -w|--workspace)
                skip_next=1
                ;;
            *)
                args+=("$arg")
                ;;
        esac
    done

    printf '%s\n' "${args[@]}"
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        -w|--workspace)
            TARGET_WS="${2:-}"
            FORWARD_ARGS+=("$1" "$2")
            shift 2
            ;;
        -p|--package|--packages)
            parse_package_list "${2:-}"
            FORWARD_ARGS+=("$1" "$2")
            shift 2
            ;;
        -j|--jobs)
            FORWARD_ARGS+=("$1" "$2")
            shift 2
            ;;
        --dry-run)
            FORWARD_ARGS+=("$1")
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
        if [ "${#TARGET_PKGS[@]}" -eq 0 ]; then
            run_agentic "${FORWARD_ARGS[@]}"
            robot_args=()
            for arg in "${FORWARD_ARGS[@]}"; do
                case "$arg" in
                    -w|--workspace|all)
                        ;;
                    *)
                        robot_args+=("$arg")
                        ;;
                esac
            done
            run_robot g1 "${robot_args[@]}"
            run_robot hexfellow "${robot_args[@]}"
            exit 0
        fi

        if [ "${#TARGET_PKGS[@]}" -ne 1 ]; then
            echo "[ERROR] package auto-detection supports one package at a time; pass --workspace for multiple packages" >&2
            exit 1
        fi

        if ! layer="$(detect_package_layer "${TARGET_PKGS[0]}")"; then
            echo "[ERROR] package not found in known build layers: ${TARGET_PKGS[0]}" >&2
            exit 1
        fi

        case "$layer" in
            agentic) run_agentic "${FORWARD_ARGS[@]}" ;;
            g1)
                mapfile -t robot_args < <(robot_forward_args "${FORWARD_ARGS[@]}")
                run_robot g1 "${robot_args[@]}"
                ;;
            hexfellow)
                mapfile -t robot_args < <(robot_forward_args "${FORWARD_ARGS[@]}")
                run_robot hexfellow "${robot_args[@]}"
                ;;
        esac
        ;;
    thirdparty|core|services)
        run_agentic "${FORWARD_ARGS[@]}"
        ;;
    g1|hexfellow)
        mapfile -t robot_args < <(robot_forward_args "${FORWARD_ARGS[@]}")
        run_robot "$TARGET_WS" "${robot_args[@]}"
        ;;
    *)
        echo "[ERROR] unknown workspace: $TARGET_WS" >&2
        echo "[ERROR] valid workspaces: all | thirdparty | core | services | g1 | hexfellow" >&2
        exit 1
        ;;
esac
