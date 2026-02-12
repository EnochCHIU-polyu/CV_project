#!/bin/bash

# Run mmpose hand detection demo with webcam on CPU
# This script assumes you have the 'openmmlab' conda environment created.

conda run -n openmmlab python demo/topdown_demo_with_mmdet.py \
    demo/mmdetection_cfg/rtmdet_nano_320-8xb32_hand.py \
    https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/rtmdet_nano_8xb32-300e_hand-267f9c8f.pth \
    configs/hand_2d_keypoint/rtmpose/hand5/rtmpose-m_8xb256-210e_hand5-256x256.py \
    https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/rtmpose-m_simcc-hand5_pt-aic-coco_210e-256x256-74fb594_20230320.pth \
    --input webcam \
    --output-root vis_results \
    --kpt-thr 0.1 \
    --device cpu \
    --show
