#!/bin/bash

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ZED_IMAGE="ghcr.io/mongsiill/zed_ros2_desktop_u22.04_sdk_5.2.0_cuda_12.6.3:latest"
BBOX_IMAGE="ghcr.io/mongsiill/yopo_3dmaker:humble"
PICKER_IMAGE="ghcr.io/mongsiill/yopo_picker:humble"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_DOMAIN_ID
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export RMW_IMPLEMENTATION
DISPLAY_ENV="${DISPLAY:-:0}"
HF_CACHE_HOST="${HF_CACHE_HOST:-$HOME/.cache/huggingface}"

# tmux 설치 확인
if ! command -v tmux &> /dev/null; then
    echo "tmux 설치 중..."
    sudo apt install -y tmux
fi

# X11 접근 허용 (Rerun/GUI 표시용)
if command -v xhost &> /dev/null; then
    xhost +local:docker >/dev/null 2>&1 || true
fi

# 이미지 없으면 pull
for IMAGE in $ZED_IMAGE $BBOX_IMAGE $PICKER_IMAGE; do
    if ! docker image inspect $IMAGE > /dev/null 2>&1; then
        echo "$IMAGE 다운로드 중..."
        docker pull $IMAGE
    fi
done

# 기존 세션 삭제
tmux kill-session -t pipeline 2>/dev/null

# docker 명령은 한 줄 (아래는 tmux에 직접 exec — send-keys 로 타이핑하지 않음)
# 레이아웃: 0.0 좌상 ZED | 0.1 우상 Picker
#           0.2 좌하 BBox ffs
# attach 시 포커스는 0.0 (ZED)

CMD_ZED="docker run --runtime nvidia -it --privileged --network=host --ipc=host --pid=host \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e DISPLAY=${DISPLAY_ENV} \
  -v /tmp/.X11-unix/:/tmp/.X11-unix \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  -v /usr/local/zed/resources/:/usr/local/zed/resources/ \
  -v /usr/local/zed/settings/:/usr/local/zed/settings/ \
  ${ZED_IMAGE} /bin/bash -lc \
  'source /opt/ros/humble/setup.bash && exec ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i'"

CMD_PICKER="docker run -it --rm --gpus all --network=host \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e DISPLAY=${DISPLAY_ENV} \
  -e NO_AT_BRIDGE=1 \
  -e YOLO_CONFIG_DIR=/root/.config/Ultralytics \
  -v /tmp/.X11-unix/:/tmp/.X11-unix \
  -v /root/.config/Ultralytics:/root/.config/Ultralytics \
  ${PICKER_IMAGE} /bin/bash -lc \
  'source /opt/ros/humble/setup.bash && source /home/user/projects/YOPO_Picker/install/setup.bash && exec ros2 run tomato_picker vlm_tomato_node'"

CMD_FFS="docker run --gpus all -it --rm --network=host \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e DISPLAY=${DISPLAY_ENV} \
  -e YOLO_CONFIG_DIR=/home/user/.config/Ultralytics \
  -e HF_HOME=/root/.cache/huggingface \
  -v /tmp/.X11-unix/:/tmp/.X11-unix \
  -v /home/user/.config/Ultralytics:/home/user/.config/Ultralytics \
  -v ${HF_CACHE_HOST}:/root/.cache/huggingface \
  ${BBOX_IMAGE} /bin/bash -lc \
  'source /opt/ros/humble/setup.bash && source /home/user/projects/YOPO_3DMaker/install/setup.bash && exec ros2 launch bbox_maker ffs_pipeline.launch.py'"

# tmux: send-keys 대신 new-session / split-window 에 실행할 명령을 직접 넘김 (셸만 뜨고 docker 안 도는 현상 방지)
tmux new-session -d -s pipeline -x 220 -y 50 bash -lc "$CMD_ZED"
sleep 8

tmux split-window -h -t pipeline:0.0 bash -lc "$CMD_PICKER"
sleep 0.5

tmux split-window -v -t pipeline:0.0 bash -lc "$CMD_FFS"

tmux select-pane -t pipeline:0.0

# tmux 세션 attach
tmux attach-session -t pipeline
