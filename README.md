# YOPO
 You Only Pick Once

# 0) (선택) 기본 도구
sudo apt update
sudo apt install -y docker.io x11-xserver-utils python3-pip
# 1) GPU 컨테이너 사용을 위한 드라이버/툴킷 확인
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
# 2) ROS2 Humble 전체 설치 (아직 안 깔았을 때만)
sudo apt install -y ros-humble-desktop
# 3) rerun_gui.py에 필요한 추가 의존(권장)
sudo apt install -y ros-humble-cv-bridge python3-opencv
# 4) Python 의존 설치
python3 -m pip install --user -r /home/user/chanyoung/YOPO/requirements.txt
