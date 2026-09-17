#!/usr/bin/env bash
### Build a Singularity image from the Docker image
### Usage: build.bash [TAG] [BUILD_ARGS...]
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
DOT_DOCKER_DIR="$(dirname "${SCRIPT_DIR}")"
REPOSITORY_DIR="$(dirname "${DOT_DOCKER_DIR}")"
IMAGES_DIR="${SCRIPT_DIR}/images"
DOCKER_BUILD_SCRIPT="${DOT_DOCKER_DIR}/build.bash"
export APPTAINER_TMPDIR="${APPTAINER_TMPDIR:-"${HOME}/.apptainer/tmp"}"

## If the current user is not in the docker group, all docker commands will be run as root
if ! grep -qi /etc/group -e "docker.*${USER}"; then
    echo "[INFO] The current user '${USER}' is not detected in the docker group. All docker commands will be run as root."
    WITH_SUDO="sudo"
fi

## Determine the name of the image to build and the output path
DOCKERHUB_USER="$(${WITH_SUDO} docker info 2>/dev/null | sed '/Username:/!d;s/.* //')"
PROJECT_NAME="$(basename "${REPOSITORY_DIR}")"
IMAGE_NAME="${DOCKERHUB_USER:+${DOCKERHUB_USER}/}${PROJECT_NAME,,}"
OUTPUT_PATH="${IMAGES_DIR}/${PROJECT_NAME}.sif"

## Parse TAG and forward additional build arguments
if [ "${#}" -gt "0" ]; then
    if [[ "${1}" != "-"* ]]; then
        TAG="${1}"
        BUILD_ARGS=${*:2}
    else
        BUILD_ARGS=${*:1}
    fi
fi
TAG="${TAG:-"latest"}"
IMAGE_NAME+=":${TAG}"

## Create the temporary directory for the Singularity image
mkdir -p "${APPTAINER_TMPDIR}"

## Build the Docker image
"${DOCKER_BUILD_SCRIPT}" "${TAG}" "${BUILD_ARGS}"

## Convert the Docker image to a Singularity image
APPTAINER_BUILD_CMD=(
    apptainer build
    "${OUTPUT_PATH}"
    "docker-daemon:${IMAGE_NAME}"
)
echo -e "\033[1;90m[TRACE] ${APPTAINER_BUILD_CMD[*]}\033[0m" | xargs
# shellcheck disable=SC2048
exec ${APPTAINER_BUILD_CMD[*]}
