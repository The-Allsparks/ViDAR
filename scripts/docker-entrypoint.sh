#!/bin/sh
set -e

if [ "${VIDAR_CAMERA_MODE:-}" = "usb" ] && [ -d /dev ]; then
  if ls /dev/video* >/dev/null 2>&1; then
    echo "[vidar] USB video devices:"
    ls -l /dev/video* || true
  else
    echo "[vidar] WARNING: VIDAR_CAMERA_MODE=usb but no /dev/video* devices found."
    echo "[vidar] On Windows, use the host camera bridge (see README)."
  fi
fi

exec "$@"
