#!/usr/bin/env bash
### Submit a job using Slurm
### Usage: submit.bash [CMD]
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
DOT_DOCKER_DIR="$(dirname "${SCRIPT_DIR}")"
REPOSITORY_DIR="$(dirname "${DOT_DOCKER_DIR}")"
IMAGES_DIR="${SCRIPT_DIR}/images"
PROJECT_NAME="$(basename "${REPOSITORY_DIR}")"

## Configuration
# Path to the image to run
IMAGE_PATH="${IMAGE_PATH:-"${IMAGES_DIR}/${PROJECT_NAME}.sif"}"
JOBS_DIR="${JOBS_DIR:-"${HOME}/jobs"}"

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

# Create a job file directory if it does not exist
if [ ! -d "${JOBS_DIR}" ]; then
    mkdir -p "${JOBS_DIR}"
    echo -e "[INFO] Created directory ${JOBS_DIR}"
fi

# Create a job file
TIMESTAMP="$(date +%Y-%m-%d_%H-%M-%S)"
JOB_PATH="${JOBS_DIR}/job_${TIMESTAMP}.bash"
cat <<EOT >"${JOB_PATH}"
#!/bin/bash -l
#SBATCH --job-name="job_${TIMESTAMP}"
#SBATCH --time=1-12:00:00
#SBATCH --qos=normal
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=168GB
#SBATCH --partition=gpu
#SBATCH --constraint volta32
#SBATCH --gpus=1
#SBATCH --gpus-per-node=1
#SBATCH --gpus-per-task=1
#SBATCH --signal=B:SIGKILL@60

bash "${SCRIPT_DIR}/run.bash" "${CMD}"
EOT

## Submit the job
SBATCH_CMD=(
    sbatch "${JOB_PATH}"
)
echo -e "\033[1;90m[TRACE] ${SBATCH_CMD[*]}\033[0m" | xargs
# shellcheck disable=SC2048
exec ${SBATCH_CMD[*]}
