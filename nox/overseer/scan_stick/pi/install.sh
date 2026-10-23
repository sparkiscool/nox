#!/usr/bin/env bash
# One-shot setup for the scan-stick Pi (Raspberry Pi 5, Raspberry Pi OS Bookworm 64-bit
# or Ubuntu 24.04 + ROS 2 Jazzy).  Run as the normal user with sudo rights:
#     bash install.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "== packages"
sudo apt-get update
sudo apt-get install -y python3-gpiozero tcpdump git cmake build-essential rpicam-apps || \
sudo apt-get install -y python3-gpiozero tcpdump git cmake build-essential libcamera-apps

echo "== data dir"
sudo mkdir -p /data/scans && sudo chown "$USER":"$USER" /data/scans

echo "== static IP for the Unitree L2 link (lidar default 192.168.1.62 -> host 192.168.1.2, UDP 6101 -> 6201)"
if command -v nmcli >/dev/null; then
  sudo nmcli con mod "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.1.2/24 2>/dev/null || \
  sudo nmcli con add type ethernet ifname eth0 con-name lidar ipv4.method manual ipv4.addresses 192.168.1.2/24
  sudo nmcli con up lidar 2>/dev/null || sudo nmcli con up "Wired connection 1" || true
fi

echo "== tcpdump without root (pcap mode)"
sudo setcap cap_net_raw,cap_net_admin=eip "$(command -v tcpdump)"

if command -v ros2 >/dev/null; then
  echo "== unilidar_sdk2 + unitree_lidar_ros2 (ROS 2 mode)"
  sudo apt-get install -y libpcl-dev ros-$ROS_DISTRO-pcl-conversions ros-$ROS_DISTRO-pcl-ros 2>/dev/null || true
  if [ ! -d "$HOME/unilidar_sdk2" ]; then
    git clone https://github.com/unitreerobotics/unilidar_sdk2.git "$HOME/unilidar_sdk2"
  fi
  (cd "$HOME/unilidar_sdk2/unitree_lidar_ros2" && source /opt/ros/*/setup.bash && colcon build)
  echo ">> lidar IP / host IP / ports live in $HOME/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py (defaults 192.168.1.62 / 192.168.1.2 / 6101 / 6201)"
else
  echo "== ros2 not found: recorder will use pcap mode (LIDAR_MODE=pcap)"
fi

echo "== systemd service"
sudo install -m 755 "$HERE/record.py" /usr/local/bin/scan-record.py
sed "s|__USER__|$USER|g; s|__HOME__|$HOME|g" "$HERE/scan-record.service" | sudo tee /etc/systemd/system/scan-record.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now scan-record.service
echo "done.  journalctl -fu scan-record  to watch it.  Button GPIO17->GND toggles, LED on GPIO27."
