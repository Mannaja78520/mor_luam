#!/bin/bash
# Use PlatformIO from its own virtualenv ($PLATFORMIO_CORE_DIR/penv), the way
# micro_ros_platformio expects. The folder is a Docker volume that may predate
# the image, so make the virtualenv if it is missing.
set -e
PENV="$PLATFORMIO_CORE_DIR/penv"
if [ ! -x "$PENV/bin/pio" ]; then
    echo "[pio] creating $PENV (first run only)"
    python -m venv "$PENV"
    "$PENV/bin/pip" install --no-cache-dir --upgrade pip platformio
fi
source "$PENV/bin/activate"
exec "$@"
