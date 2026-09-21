#!/usr/bin/env bash
set -e

HOST_DIR="/host-nvidia-libs"
TARGET_DIR="/usr/lib/x86_64-linux-gnu"

# Verify the host library folder was mounted
if [ ! -d "$HOST_DIR" ]; then
    echo "Warning: $HOST_DIR not found. Skipping Nvidia library setup."
    exit 0
fi

echo "Linking Nvidia driver libraries from host..."

# Symlink all Nvidia-related dynamic libraries into the standard system path
for file in "$HOST_DIR"/*nvidia* "$HOST_DIR"/*EGL_nvidia* "$HOST_DIR"/*GLES* "$HOST_DIR"/*GLX_nvidia*; do
    if [ -e "$file" ]; then
        filename=$(basename "$file")
        ln -sf "$file" "$TARGET_DIR/$filename"
    fi
done

# Rebuild the dynamic linker run-time bindings cache
ldconfig

echo "Nvidia library symlinks updated successfully."
