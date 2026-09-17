#!/usr/bin/env bash
### Run a command inside the Singularity container
### Usage: run.bash [CMD]
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
DOT_DOCKER_DIR="$(dirname "${SCRIPT_DIR}")"
REPOSITORY_DIR="$(dirname "${DOT_DOCKER_DIR}")"
IMAGES_DIR="${SCRIPT_DIR}/images"
PROJECT_NAME="$(basename "${REPOSITORY_DIR}")"

## Configuration
# Path to the image to run
IMAGE_PATH="${IMAGE_PATH:-"${IMAGES_DIR}/${PROJECT_NAME}.sif"}"
# Options for running the container
SINGULARITY_EXEC_OPTS="${SINGULARITY_EXEC_OPTS:-
    --no-home
}"
# Flag to enable GPU
WITH_GPU="${WITH_GPU:-true}"
# Volumes to mount inside the container
VOLUMES_ROOT="${SCRATCH:-${HOME}}/volumes/${PROJECT_NAME}"
SINGULARITY_VOLUMES=(
    ## Isaac Sim
    # Data
    "${VOLUMES_ROOT}/.nvidia-omniverse/data/ov:/root/.local/share/ov/data:rw"
    "${VOLUMES_ROOT}/.nvidia-omniverse/data/isaac-sim:/root/isaac-sim/kit/data:rw"
    # Cache
    "${VOLUMES_ROOT}/.cache/isaac-sim:/root/isaac-sim/kit/cache:rw"
    "${VOLUMES_ROOT}/.cache/nvidia/GLCache:/root/.cache/nvidia/GLCache:rw"
    "${VOLUMES_ROOT}/.cache/ov:/root/.cache/ov:rw"
    "${VOLUMES_ROOT}/.nv/ComputeCache:/root/.nv/ComputeCache:rw"
    # Logs
    "${VOLUMES_ROOT}/.nvidia-omniverse/logs:/root/.nvidia-omniverse/logs:rw"
    "${VOLUMES_ROOT}/.nvidia-omniverse/logs/isaac-sim:/root/isaac-sim/kit/logs:rw"
    ## SimForge
    # Cache
    "${VOLUMES_ROOT}/.cache/simforge:/root/.cache/simforge:rw"
    ## Project
    # Source
    "${REPOSITORY_DIR}:/root/ws:rw"
    ## Misc
    "${VOLUMES_ROOT}/home/users:/home/users:rw"
)

## Ensure the image exists
if [ ! -f "${IMAGE_PATH}" ]; then
    echo >&2 -e "\033[1;31m[ERROR] Singularity image not found at ${IMAGE_PATH}\033[0m"
    exit 1
fi

## Parse CMD
if [ "${#}" -gt "0" ]; then
    CMD=${*:1}
else
    echo >&2 -e "\033[1;31m[ERROR] No command provided.\033[0m"
    exit 1
fi

## Ensure the host directories exist
for volume in "${SINGULARITY_VOLUMES[@]}"; do
    if [[ "${volume}" =~ ^([^:]+):([^:]+).*$ ]]; then
        host_dir="${BASH_REMATCH[1]}"
        if [ ! -d "${host_dir}" ]; then
            mkdir -p "${host_dir}"
            echo -e "[INFO] Created directory ${host_dir}"
        fi
    fi
done

## GPU
if [[ "${WITH_GPU,,}" = true ]]; then
    SINGULARITY_EXEC_OPTS+=" --nv"
fi

## Environment
if ! command -v module >/dev/null 2>&1; then
    echo >&2 -e "\033[1;31m[ERROR] The 'module' command is not available. Please run this script on a compute node.\033[0m"
    exit 1
fi
# Load the Singularity module
module purge
module load tools/Singularity

## Run the container
SINGULARITY_EXEC_CMD=(
    singularity exec
    "${SINGULARITY_EXEC_OPTS}"
    "${SINGULARITY_VOLUMES[@]/#/"--bind "}"
    "${IMAGE_PATH}"
    "${CMD}"
)
echo -e "\033[1;90m[TRACE] ${SINGULARITY_EXEC_CMD[*]}\033[0m" | xargs
# shellcheck disable=SC2048
exec ${SINGULARITY_EXEC_CMD[*]}
