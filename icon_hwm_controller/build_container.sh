#!/bin/bash
set -e

# Ensure we are at the root of the workspace
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/../../.." &> /dev/null && pwd )"
cd "$WORKSPACE_DIR"

STAGING_DIR="container_staging"

echo "Cleaning up old staging directory..."
rm -rf "$STAGING_DIR"
mkdir -p "$STAGING_DIR"

echo "Copying files to staging..."
PACKAGES=(
    "icon_hwm_controller"
    "icon_hwm_controller_msgs"
    "icon_shared_memory_vendor"
    "flatbuffers_vendor"
    "eigen_vendor"
)

for pkg in "${PACKAGES[@]}"; do
    if [ -d "install/$pkg" ]; then
        echo "Copying $pkg..."
        distrobox-enter -n kilted-osrf -- cp -rL "install/$pkg" "$STAGING_DIR/"
    else
        echo "Warning: install/$pkg not found!"
    fi
done

echo "Copying setup files..."
distrobox-enter -n kilted-osrf -- cp -L install/local_setup.* "$STAGING_DIR/"
distrobox-enter -n kilted-osrf -- cp -L install/setup.* "$STAGING_DIR/"
distrobox-enter -n kilted-osrf -- cp -L install/_local_setup_util_*.py "$STAGING_DIR/"

echo "Building base docker container..."
docker build --load -t icon_hwm_base:latest -f src/sdk-ros/icon_hwm_controller/Dockerfile.base src/sdk-ros/icon_hwm_controller

echo "Building docker container..."
docker build --load -t icon_hwm -f src/sdk-ros/icon_hwm_controller/Dockerfile "$STAGING_DIR"

echo "Cleaning up staging dir"
rm -rf "$STAGING_DIR"
echo "Done!"
