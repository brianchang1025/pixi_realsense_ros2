#!/usr/bin/env bash
set -euo pipefail

RQT_CMD="pixi run -e jazzy rqt --perspective-file ./scripts/cameraviewer.perspective"


echo "Launching rqt: $RQT_CMD"
# Start rqt
eval "$RQT_CMD"
