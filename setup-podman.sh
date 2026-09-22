#!/usr/bin/env bash
#
# @author: Pedro Ribeiro
# @modified: 22/09/2026
#
# This script configures Podman and VS Code for the CS labs, namely by:
#
# 1. Configuring the file ~/.config/containers.conf, so that the mapping of UID/GID
#    from the container maps the user 'ubuntu' (uid:1000,gid:1000) as own user,
#    rather than mapping 'root'. This facilitates permissions to access resources
#    shared by the host machine, including X11 and the network file share.
#
# 2. Configuring the file ~/.config/storage.conf, so that the graphroot, where 
#    Podman stores images/volumes is /scratch/$USER/containers.
#
# 3. Enables podman's socket via a user-level systemd configuration, ie. by running:
#       systemctl --user enable --now podman.socket
#
# 4. Adds an autostart desktop entry that creates a symlink from
#    ~/.Xauthority to $XAUTHORITY. On wayland and similar, this $XAUTHORITY
#    lives at an unpredictable location, so we use the symlink to ensure the container
#    only has to mount from a predictable location at ~/.Xauthority
#
# 5. Configures VS Code's Dev Container extensions to use podman instead of Docker,
#    by changing two VS Code settings:
#
#       dev.containers.dockerPath -> podman
#       dev.containers.dockerSocketPath -> $XDG_RUNTIME_DIR/podman/podman.sock
#
set -euo pipefail

CONFIG_DIR="$HOME/.config/containers"
AUTOSTART_DIR="$HOME/.config/autostart"
DESKTOP_FILE="$AUTOSTART_DIR/fix-xauthority.desktop"
VSCODE_DIR="$HOME/.config/Code/User"

configure_vscode() {
    echo "Configuring VS Code settings..."
    mkdir -p "$VSCODE_DIR"

    python3 - <<'EOF'
import json
import os

xdg_runtime = os.environ.get("XDG_RUNTIME_DIR") or f"/run/user/{os.getuid()}"
socket_path = os.path.join(xdg_runtime, "podman/podman.sock")
settings_path = os.path.expanduser("~/.config/Code/User/settings.json")

new_settings = {
    "dev.containers.dockerPath": "podman",
    "dev.containers.dockerSocketPath": socket_path
}

data = {}
if os.path.exists(settings_path):
    try:
        with open(settings_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except Exception:
        data = {}

data.update(new_settings)

with open(settings_path, 'w', encoding='utf-8') as f:
    json.dump(data, f, indent=4)

print(f"VS Code settings updated (dockerSocketPath = {socket_path})")
EOF
}

install_podman() {
    echo "Installing Podman user configuration..."

    mkdir -p "$CONFIG_DIR" "$AUTOSTART_DIR"

    # 1. Write storage.conf
    cat <<'EOF' > "$CONFIG_DIR/storage.conf"
[storage]
driver = "overlay"
graphroot = "/scratch/$USER/containers"
EOF

    # 2. Write containers.conf
    cat <<'EOF' > "$CONFIG_DIR/containers.conf"
[containers]
userns = "keep-id"
EOF

    # 3. Enable podman socket
    if command -v systemctl &>/dev/null; then
        systemctl --user enable --now podman.socket
    fi

    # 4. Create XDG Autostart entry for Xauthority
    cat <<'EOF' > "$DESKTOP_FILE"
[Desktop Entry]
Type=Application
Name=Fix Xauthority Symlink
Exec=sh -c 'if [ -n "$XAUTHORITY" ]; then ln -fs "$XAUTHORITY" "$HOME/.Xauthority"; fi'
Hidden=false
NoDisplay=true
X-GNOME-Autostart-enabled=true
EOF

    if [ -n "${XAUTHORITY:-}" ]; then
        ln -fs "$XAUTHORITY" "$HOME/.Xauthority"
    fi

    # 5. Configure VS Code settings
    if command -v python3 &>/dev/null; then
        configure_vscode
    else
        echo "Warning: python3 not found. Skipping VS Code auto-configuration."
    fi

    echo "Podman configuration installed successfully."

    verify_podman
}

uninstall_podman() {
    echo "Uninstalling Podman user configuration..."

    if command -v systemctl &>/dev/null; then
        systemctl --user disable --now podman.socket 2>/dev/null || true
    fi

    rm -f "$CONFIG_DIR/storage.conf"
    rm -f "$CONFIG_DIR/containers.conf"
    rm -f "$DESKTOP_FILE"
    rm -f "$HOME/.Xauthority"

    rmdir "$CONFIG_DIR" 2>/dev/null || true
    rmdir "$AUTOSTART_DIR" 2>/dev/null || true

    echo "Podman configuration uninstalled successfully."
}

verify_podman() {
    echo "Testing Podman configuration..."

    # Check container UID (expects 1000 due to keep-id:uid=1000,gid=1000)
    CONTAINER_UID=$(podman run --rm docker.io/library/alpine:latest id -u 2>/dev/null || echo "error")

    if [ "$CONTAINER_UID" = "1000" ]; then
        echo "✅ Success: Podman is configured correctly (Container UID is 1000)."
    else
        echo "❌ Failure: Test container failed. Run this command manually to inspect:"
        echo "  podman run --rm docker.io/library/alpine:latest id -u"
    fi
}

case "${1:-}" in
    install)
        install_podman
        ;;
    uninstall)
        uninstall_podman
        ;;
    *)
        echo "Usage: $0 {install|uninstall}"
        exit 1
        ;;
esac
