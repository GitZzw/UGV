#!/bin/bash
source /opt/intel/openvino_2020.4.287/bin/setupvars.sh
### zzw_nodes_run
cd ~/zzw/Nanodet
#conda activate zzw4
python3 d435msg.py webcam --model model/model_last.pth --config config/test.yml  --path 0 & sleep 10
python client.py
exit 0
