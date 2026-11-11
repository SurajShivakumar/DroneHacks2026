#!/usr/bin/env bash
# Install a verified portable GCS in user-owned runtime. Does not start GCS/flight.
set -euo pipefail
folder="$HOME/dronehacks-runtime/qgc-v5.1.5"
mkdir -p "$folder"
cd "$folder"
name=QGroundControl-aarch64.AppImage
if [ ! -f "$name" ]; then
    curl -fsSL --retry 3 https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.5/QGroundControl-aarch64.AppImage -o "$name.part"
    printf '%s\n' 'ddd92fe3a1cd1c1c21a3028052e9f9b4d4a68cb80689552d3f34727ed2c91e03  QGroundControl-aarch64.AppImage.part' | sha256sum -c -
    mv "$name.part" "$name"
fi
printf '%s\n' 'ddd92fe3a1cd1c1c21a3028052e9f9b4d4a68cb80689552d3f34727ed2c91e03  QGroundControl-aarch64.AppImage' | sha256sum -c -
chmod u+x "$name"
if [ ! -d squashfs-root ]; then
    ./"$name" --appimage-extract > extract.log
fi
printf '\nPortable GCS extracted; no application or flight started.\n'
