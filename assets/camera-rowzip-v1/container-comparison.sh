#!/bin/sh
set -eu
cd /workspace/rust_drive
python -B integrations/onnx/camera_rowzip_check.py controls --output artifacts/camera-rowzip-v1/controls
python -B integrations/onnx/camera_rowzip_check.py run --controls artifacts/camera-rowzip-v1/controls --output artifacts/camera-rowzip-v1/validation
