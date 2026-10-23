#!/usr/bin/env python3
"""
Noxen scan stick recorder.

One button (GPIO17 -> GND) toggles a recording session; the LED (GPIO27)
shows state:  off = idle,  solid = recording,  fast blink = error.

Each session writes to  /data/scans/<YYYYmmdd_HHMMSS>/ :
    lidar/        ROS 2 bag (unitree_lidar_ros2: unilidar/cloud + unilidar/imu)  [mode=ros2]
    lidar.pcap    raw Unitree L2 UDP capture (lidar 192.168.1.62 -> host 192.168.1.2:6201) [mode=pcap]
    cam.h264      Pi camera H.264 stream
    cam.pts       per-frame timestamps (ms) for cam.h264
    meta.json     start/stop wall-clock + monotonic times, so streams can be aligned

Mode is picked with LIDAR_MODE=ros2|pcap (default: ros2 if `ros2` is on PATH, else pcap).
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

DATA_ROOT = Path(os.environ.get("SCAN_DATA_ROOT", "/data/scans"))
LIDAR_MODE = os.environ.get("LIDAR_MODE") or ("ros2" if shutil.which("ros2") else "pcap")
LIDAR_IFACE = os.environ.get("LIDAR_IFACE", "eth0")
ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/jazzy/setup.bash")
LIDAR_WS = os.environ.get("LIDAR_WS", str(Path.home() / "unilidar_sdk2" / "unitree_lidar_ros2"))
LIDAR_IP = os.environ.get("LIDAR_IP", "192.168.1.62")
CAM_ARGS = os.environ.get("CAM_ARGS", "--width 1920 --height 1080 --framerate 30 --codec h264 --bitrate 8000000")
BUTTON_PIN = int(os.environ.get("BUTTON_PIN", "17"))
LED_PIN = int(os.environ.get("LED_PIN", "27"))

try:
    from gpiozero import Button, LED
except ImportError:  # allow running on a desktop for a dry run
    Button = LED = None


class Session:
    def __init__(self):
        self.dir = None
        self.procs = {}
        self.meta = {}

    def start(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = DATA_ROOT / stamp
        self.dir.mkdir(parents=True, exist_ok=True)
        self.meta = {
            "start_wall": time.time(),
            "start_monotonic": time.monotonic(),
            "lidar_mode": LIDAR_MODE,
            "cam_args": CAM_ARGS,
        }
        # ---- camera ----
        cam_cmd = (
            f"rpicam-vid -t 0 -n {CAM_ARGS} "
            f"-o {self.dir/'cam.h264'} --save-pts {self.dir/'cam.pts'}"
        )
        self.procs["cam"] = subprocess.Popen(cam_cmd, shell=True, preexec_fn=os.setsid)

        # ---- lidar ----
        if LIDAR_MODE == "ros2":
            # driver + bag in one shell so the ROS env is sourced once
            lidar_cmd = (
                f"source {ROS_SETUP} && source {LIDAR_WS}/install/setup.bash && "
                f"ros2 launch unitree_lidar_ros2 launch.py & "
                f"sleep 3 && ros2 bag record -o {self.dir/'lidar'} unilidar/cloud unilidar/imu; wait"
            )
            self.procs["lidar"] = subprocess.Popen(["bash", "-c", lidar_cmd], preexec_fn=os.setsid)
        else:
            # raw capture of everything the L2 sends (UDP from 192.168.1.62:6101 to host :6201)
            lidar_cmd = f"tcpdump -i {LIDAR_IFACE} -w {self.dir/'lidar.pcap'} udp and src host {LIDAR_IP}"
            self.procs["lidar"] = subprocess.Popen(lidar_cmd, shell=True, preexec_fn=os.setsid)

        (self.dir / "meta.json").write_text(json.dumps(self.meta, indent=2))
        print(f"[rec] started {self.dir}", flush=True)

    def stop(self):
        for name, p in self.procs.items():
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        for name, p in self.procs.items():
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        self.meta["stop_wall"] = time.time()
        self.meta["stop_monotonic"] = time.monotonic()
        (self.dir / "meta.json").write_text(json.dumps(self.meta, indent=2))
        print(f"[rec] stopped {self.dir}", flush=True)
        self.procs = {}

    def healthy(self):
        return all(p.poll() is None for p in self.procs.values())


def main():
    session = Session()
    recording = False
    led = LED(LED_PIN) if LED else None
    btn = Button(BUTTON_PIN, pull_up=True, bounce_time=0.05) if Button else None

    def toggle():
        nonlocal recording
        if not recording:
            session.start()
            recording = True
            if led: led.on()
        else:
            session.stop()
            recording = False
            if led: led.off()

    if btn:
        btn.when_pressed = toggle
    else:
        print("gpiozero not available: press Enter to toggle recording")

    def on_exit(*_):
        if recording:
            session.stop()
        sys.exit(0)

    signal.signal(signal.SIGTERM, on_exit)
    signal.signal(signal.SIGINT, on_exit)

    while True:
        if btn is None:
            sys.stdin.readline()
            toggle()
            continue
        time.sleep(0.5)
        if recording and not session.healthy():
            # a child died (lidar unplugged, camera busy, disk full ...): stop and blink
            session.stop()
            recording = False
            if led: led.blink(0.1, 0.1)


if __name__ == "__main__":
    main()
