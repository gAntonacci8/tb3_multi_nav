#!/usr/bin/env bash
set -e

xhost +local:root >/dev/null 2>&1 || true

docker run -it --rm \
  --name tag_game \
  --network host \
  --ipc host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  -v /usr/lib/wsl:/usr/lib/wsl \
  --device=/dev/dxg \
  -e LD_LIBRARY_PATH=/usr/lib/wsl/lib:/usr/lib/wsl/lib64:$LD_LIBRARY_PATH \
  -e LIBGL_DRIVERS_PATH=/usr/lib/wsl/lib/dri \
  -e __GLX_VENDOR_LIBRARY_NAME=mesa \
  -e MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA" \
  -e GALLIUM_DRIVER=d3d12 \
  -e QT_X11_NO_MITSHM=1 \
  -e TURTLEBOT3_MODEL=waffle \
  -v "$PWD/ros_ws":/root/ros_ws \
  -v "$PWD/launch":/root/launch \
  -e GZ_RENDER_ENGINE=ogre2 \
  -e GZ_SIM_MSAA=0 \
  -e GZ_SIM_RESOURCE_PATH="/root/ros_ws/src" \
  ros:tag_game bash
