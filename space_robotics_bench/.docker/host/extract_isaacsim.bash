#!/usr/bin/env bash
### Extract Isaac Sim from a Docker image
### Usage: extract_isaacsim.bash [destination_directory]
set -e

## Config
SRC_IMAGE="${SRC_IMAGE:-"andrejorsula/space_robotics_bench:latest"}"
SRC_PATH="${SRC_PATH:-"/root/isaac-sim"}"
DEST_DIR="${1:-"${DEST_DIR:-"$HOME/isaac-sim"}"}"
CONTAINER_NAME="isaac_sim_extract_$(date +%s)"

echo "[INFO] Source image: ${SRC_IMAGE}"
echo "[INFO] Source path: ${SRC_PATH}"
echo "[INFO] Destination path: ${DEST_DIR}"

# Check if the destination directory already exists and prompt for overwrite
if [[ -d "${DEST_DIR}" ]]; then
    echo -en "\033[1;33m[WARNING] Destination directory already exists, overwrite? [y/N]\033[0m "
    read -r
    if [[ ! "${REPLY}" =~ ^[Yy]$ ]]; then
        echo "[INFO] Exiting"
        exit 0
    fi
fi

# Ensure Docker is running
if ! docker info >/dev/null 2>&1; then
    echo >&2 -e "\033[1;31m[ERROR] Docker is not running or not accessible\033[0m"
    exit 1
fi

# Check if the image is available locally, pull if necessary
if [[ -z "$(docker image ls -q "${SRC_IMAGE}")" ]]; then
    echo "[INFO] Pulling the Docker image"
    docker pull "${SRC_IMAGE}" &>/dev/null
else
    echo "[INFO] Docker image is already available locally"
fi

# Function to clean up the container on exit
cleanup() {
    echo "[INFO] Stopping container"
    docker stop "${CONTAINER_NAME}" &>/dev/null || true
}
trap cleanup EXIT  # Ensure cleanup runs on script exit

# Start Docker container in the background
echo "[INFO] Starting a temporary Docker container: ${CONTAINER_NAME}"
docker run --name "${CONTAINER_NAME}" --rm --entrypoint bash "${SRC_IMAGE}" -c "sleep infinity" &

# Wait for the container to be ready
echo "[INFO] Checking if container is ready"
for i in {1..10}; do
    sleep 0.5
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        if docker exec "${CONTAINER_NAME}" ls "${SRC_PATH}" &>/dev/null; then
            echo "[INFO] Container is ready"
            break
        else
            echo >&2 -e "\033[1;31m[ERROR] Path '${SRC_PATH}' not found in container '${SRC_IMAGE}'\033[0m"
            exit 1
        fi
    fi
    if [[ "${i}" -eq 10 ]]; then
        echo >&2 -e "\033[1;31m[ERROR] Container did not start in time\033[0m"
        exit 1
    fi
done

# Ensure the destination directory exists
mkdir -p "$(dirname "${DEST_DIR}")"

# Copy the directory from the container
echo "[INFO] Copying ${CONTAINER_NAME}:${SRC_PATH} to ${DEST_DIR}"
docker cp "${CONTAINER_NAME}:${SRC_PATH}" "${DEST_DIR}"

# Update pip in the extracted environment
echo "[INFO] Updating pip in extracted environment"
if [[ -f "${DEST_DIR}/python.sh" ]]; then
    "${DEST_DIR}/python.sh" -m pip install --upgrade pip
else
    echo >&2 -e "\033[1;33m[WARNING] python.sh not found in ${DEST_DIR}, skipping pip upgrade\033[0m"
fi

# Print environment setup instructions
echo "[INFO] Isaac Sim has been extracted to ${DEST_DIR}"
echo "[INFO] Recommended environment variable setup:"
echo "[INFO]   echo \"export ISAAC_SIM_PYTHON='${DEST_DIR}/python.sh'\" >> ~/.bashrc"
echo "[INFO]   set -Ux ISAAC_SIM_PYTHON '${DEST_DIR}/python.sh' # Fish"
