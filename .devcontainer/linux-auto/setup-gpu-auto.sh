#!/usr/bin/env bash
set -e

HOST_ROOT="/host-root"
HOST_DEV="$HOST_ROOT/dev"
HOST_LIBS="$HOST_ROOT/usr/lib/x86_64-linux-gnu"
TARGET_LIBS="/usr/lib/x86_64-linux-gnu"

# 1. Unconditionally link /dev/dri if present on the host
if [ -d "$HOST_DEV/dri" ] && [ ! -d "/dev/dri" ]; then
    echo "Linking /dev/dri for DRM/Mesa/EGL rendering..."
    ln -sf "$HOST_DEV/dri" /dev/dri
fi

# 2. Check for NVIDIA driver availability
if compgen -G "$HOST_DEV/nvidia*" > /dev/null || [ -f "$HOST_ROOT/usr/bin/nvidia-smi" ]; then
    echo "NVIDIA driver detected. Linking devices, binaries, and libraries..."

    # Symlink NVIDIA devices
    for dev in "$HOST_DEV"/nvidia*; do
        if [ -e "$dev" ]; then
            devname=$(basename "$dev")
            ln -sf "$dev" "/dev/$devname"
        fi
    done

    # Symlink Host Binaries
    [ -f "$HOST_ROOT/usr/bin/nvidia-smi" ] && ln -sf "$HOST_ROOT/usr/bin/nvidia-smi" /usr/bin/nvidia-smi
    [ -f "$HOST_ROOT/usr/bin/nvidia-persistenced" ] && ln -sf "$HOST_ROOT/usr/bin/nvidia-persistenced" /usr/bin/nvidia-persistenced

    # Symlink Vendor ICDs
    [ -f "$HOST_ROOT/etc/OpenCL/vendors/nvidia.icd" ] && { mkdir -p /etc/OpenCL/vendors; ln -sf "$HOST_ROOT/etc/OpenCL/vendors/nvidia.icd" /etc/OpenCL/vendors/nvidia.icd; }
    [ -f "$HOST_ROOT/usr/share/glvnd/egl_vendor.d/10_nvidia.json" ] && { mkdir -p /usr/share/glvnd/egl_vendor.d; ln -sf "$HOST_ROOT/usr/share/glvnd/egl_vendor.d/10_nvidia.json" /usr/share/glvnd/egl_vendor.d/10_nvidia.json; }
    [ -f "$HOST_ROOT/usr/share/vulkan/icd.d/nvidia_icd.json" ] && { mkdir -p /etc/vulkan/icd.d; ln -sf "$HOST_ROOT/usr/share/vulkan/icd.d/nvidia_icd.json" /etc/vulkan/icd.d/nvidia_icd.json; }
    [ -e "$HOST_ROOT/run/nvidia-persistenced/socket" ] && { mkdir -p /run/nvidia-persistenced; ln -sf "$HOST_ROOT/run/nvidia-persistenced/socket" /run/nvidia-persistenced/socket; }

    # Symlink Host Libraries
    if [ -d "$HOST_LIBS" ]; then
        for file in "$HOST_LIBS"/*nvidia* "$HOST_LIBS"/*EGL_nvidia* "$HOST_LIBS"/*GLES* "$HOST_LIBS"/*GLX_nvidia*; do
            if [ -e "$file" ]; then
                filename=$(basename "$file")
                ln -sf "$file" "$TARGET_LIBS/$filename"
            fi
        done
    fi

# Write GPU profile overrides to force NVIDIA GLVND and silence Mesa warnings
cat << 'EOF' > /etc/profile.d/nvidia-gpu.sh
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __NV_PRIME_RENDER_OFFLOAD=1
export EGL_LOG_LEVEL=fatal
EOF
chmod +x /etc/profile.d/nvidia-gpu.sh

    # Insert hook AT LINE 1 of /etc/bash.bashrc (before [ -z "$PS1" ] && return)
    if ! grep -q "nvidia-gpu.sh" /etc/bash.bashrc; then
        sed -i '1i [ -f /etc/profile.d/nvidia-gpu.sh ] && . /etc/profile.d/nvidia-gpu.sh' /etc/bash.bashrc
    fi

    # Populate /etc/environment for system-wide defaults
    sed -i '/__GLX_VENDOR_LIBRARY_NAME/d' /etc/environment
    sed -i '/__EGL_VENDOR_LIBRARY_FILENAMES/d' /etc/environment
    sed -i '/__NV_PRIME_RENDER_OFFLOAD/d' /etc/environment
    sed -i '/EGL_LOG_LEVEL/d' /etc/environment
    cat << 'EOF' >> /etc/environment
__GLX_VENDOR_LIBRARY_NAME=nvidia
__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
__NV_PRIME_RENDER_OFFLOAD=1
EGL_LOG_LEVEL=fatal
EOF

    ldconfig
    echo "NVIDIA GPU auto-configuration complete."
else
    echo "Standard DRI rendering active."
    rm -f /etc/profile.d/nvidia-gpu.sh
fi

# Detect host Xauthority source file
TARGET_XAUTH=""

if [ -n "$HOST_XAUTHORITY" ] && [ -e "/host-root${HOST_XAUTHORITY}" ]; then
    TARGET_XAUTH="/host-root${HOST_XAUTHORITY}"
elif [ -n "$HOST_HOME" ] && [ -e "/host-root${HOST_HOME}/.Xauthority" ]; then
    TARGET_XAUTH="/host-root${HOST_HOME}/.Xauthority"
elif [ -e "/home/ubuntu/homedir/.Xauthority" ]; then
    TARGET_XAUTH="/home/ubuntu/homedir/.Xauthority"
else
    # Extended search pattern to match Xwayland, mutter, and standard xauth files
    SEARCH_RESULT=$(sudo find /host-root/run/user /host-root/tmp -type f \( -name "*Xauthority*" -o -name "*Xwaylandauth*" -o -name "xauth_*" \) 2>/dev/null | head -n 1)
    if [ -n "$SEARCH_RESULT" ]; then
        TARGET_XAUTH="$SEARCH_RESULT"
    fi
fi

# Merge host keys into container .Xauthority without breaking local VNC keys
if [ -n "$TARGET_XAUTH" ]; then
    echo "Merging host Xauthority from $TARGET_XAUTH..."
    touch /home/ubuntu/.Xauthority
    chown ubuntu:ubuntu /home/ubuntu/.Xauthority 2>/dev/null || true
    sudo -u ubuntu xauth merge "$TARGET_XAUTH" 2>/dev/null || true
else
    echo "No host Xauthority file detected."
fi