#!/usr/bin/env bash

# Parse X11 Display Socket
disp="${DISPLAY:-:0}"
disp_nohost="${disp##*:}"
dispnum="${disp_nohost%%.*}"
socket_path="/tmp/.X11-unix/X${dispnum}"

xauth_path="${XAUTHORITY:-$HOME/.Xauthority}"

# 1. Update Xauthority permissions
if [ -f "$xauth_path" ]; then
    # Try setfacl first, fallback to chmod, ignore failure if already readable
    sudo setfacl -m "u:$(id -u):r" "$xauth_path" 2>/dev/null \
        || chmod +r "$xauth_path" 2>/dev/null \
        || true
fi

# 2. Update X11 Socket permissions
if [ -e "$socket_path" ]; then
    # Only attempt setfacl if the current user doesn't already have write access
    if [ ! -w "$socket_path" ]; then
        sudo setfacl -m "u:$(id -u):rw" "$socket_path" 2>/dev/null || true
    fi

    # Final sanity check: verify socket is writable
    if [ ! -w "$socket_path" ]; then
        echo "Warning: Socket $socket_path is not writable. GUI applications may fail to open."
    else
        echo "X11 socket $socket_path is ready."
    fi
else
    echo "Warning: X11 socket $socket_path does not exist."
fi