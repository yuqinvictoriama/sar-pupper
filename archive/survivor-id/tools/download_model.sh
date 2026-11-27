#!/bin/bash
# Fetch the pose model survivor identification runs on the Hailo-8L (about 25 MB).
#   bash tools/download_model.sh          yolov8s_pose (the default in config/survivor.yaml)
#   bash tools/download_model.sh m        yolov8m_pose (more accurate, about half the speed; plan decision S3)
# Same model-zoo release as follow_me_pupper's yolov8n_seg.hef, so it matches the robot's HailoRT.
set -e
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
SIZE=${1:-s}
mkdir -p "$ROOT/models"; cd "$ROOT/models"
wget -nc "https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8l/yolov8${SIZE}_pose.hef"
echo "Done -> $ROOT/models/yolov8${SIZE}_pose.hef"
echo "Check it loads and how fast it runs:  python3 tools/bench_hailo.py"
