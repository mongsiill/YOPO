#!/bin/bash

ZED_IMAGE="ghcr.io/mongsiill/zed_ros2_desktop_u22.04_sdk_5.2.0_cuda_12.6.3:latest"
BBOX_IMAGE="ghcr.io/mongsiill/edgetam-bbox:humble"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
DISPLAY_ENV="${DISPLAY:-:0}"

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
for IMAGE in $ZED_IMAGE $BBOX_IMAGE; do
    if ! docker image inspect $IMAGE > /dev/null 2>&1; then
        echo "$IMAGE 다운로드 중..."
        docker pull $IMAGE
    fi
done

# 기존 세션 삭제
tmux kill-session -t pipeline 2>/dev/null

# tmux 세션 시작
tmux new-session -d -s pipeline -x 220 -y 50

# 터미널 1 - ZED 카메라
tmux send-keys -t pipeline "docker run --runtime nvidia -it --privileged \
    --network=host --ipc=host --pid=host \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY_ENV \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    -v /usr/local/zed/resources/:/usr/local/zed/resources/ \
    -v /usr/local/zed/settings/:/usr/local/zed/settings/ \
    $ZED_IMAGE \
    /bin/bash -c 'source /opt/ros/humble/setup.bash && \
    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i'" Enter

# ZED 완전히 뜰 때까지 대기
echo "ZED 카메라 시작 대기 중..."
sleep 5

# 터미널 2 - Rerun Bridge (BBox 컨테이너 내부)
tmux split-window -h -t pipeline
tmux send-keys -t pipeline "docker run --gpus all -it --rm \
    --network=host \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e DISPLAY=$DISPLAY_ENV \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    $BBOX_IMAGE \
    /bin/bash -c 'source /opt/ros/humble/setup.bash && \
    source /home/user/projects/EdgeTAM_bbox/install/setup.bash && \
    ros2 run bbox_maker rerun_bridge_node --ros-args \
    -p world_frame:=map \
    -p lock_to_world:=true \
    -p log_camera_pose:=true \
    -p camera_entity_path:=world/camera \
    -p max_points:=0 \
    -p rerun_spawn:=true'" Enter

# 터미널 3 - BBox Pipeline
tmux split-window -v -t pipeline:0.1
tmux send-keys -t pipeline:0.2 "docker run --gpus all -it --rm \
    --network=host \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e DISPLAY=$DISPLAY_ENV \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    $BBOX_IMAGE \
    /bin/bash -c 'source /opt/ros/humble/setup.bash && \
    source /home/user/projects/EdgeTAM_bbox/install/setup.bash && \
    ros2 launch bbox_maker ffs_pipeline.launch.py'" Enter

# tmux 세션 attach
tmux attach-session -t pipeline