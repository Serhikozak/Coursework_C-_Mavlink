#!/bin/bash

cd /mnt/c
/mnt/c/Windows/System32/cmd.exe /c start "" cmd.exe /k "C:/PROGRA~1/WSL/wsl.exe -d Ubuntu-24.04 --cd /home/docker/MavLink_Autopilot bash -ic 'dclogs;exec bash'"
sleep 1
/mnt/c/Windows/System32/cmd.exe /c start "" cmd.exe /k "C:/PROGRA~1/WSL/wsl.exe -d Ubuntu-24.04 --cd /home/docker/MavLink_Autopilot bash -ic 'python3 BridgeGaz.py; exec bash'"
sleep 1
/mnt/c/Windows/System32/cmd.exe /c start "" cmd.exe /k "C:/PROGRA~1/WSL/wsl.exe -d Ubuntu-24.04 --cd /home/docker/MavLink_Autopilot bash -ic 'gz sim -s -v4 -r iris_runway.sdf; exec bash'"
sleep 1
/mnt/c/Windows/System32/cmd.exe /c start "" cmd.exe /k "C:/PROGRA~1/WSL/wsl.exe -d Ubuntu-24.04 --cd /home/docker/MavLink_Autopilot bash -ic 'gz topic -t "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming" -m gz.msgs.Boolean -p "data: 1";exec bash'"



